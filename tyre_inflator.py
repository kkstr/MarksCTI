#!/usr/bin/env python3
"""
4WD Tyre Pressure Monitor & Auto-Inflator — Raspberry Pi 5

Full system overview, wiring, lockout/warning behaviour and setup notes
live in OVERVIEW.txt next to this script. Quick summary:

- Per-corner pressure monitoring via an ADS1115 ADC (0.5-4.5V = 0-150psi).
- Automatic inflate/deflate to ROAD/DIRT/SAND/CUSTOM presets (front/rear
  targets, persisted to tyre_presets.json).
- SSD1309 OLED dashboard with a 4WD outline + flashing puncture /
  compressor-failure warnings.
- GPS speed-based 2-stage control lockout (UI only — regulation never
  stops; fail-safe to the most restrictive stage with no fix).
- Demand-based compressor control and self-healing sensor fault detection
  (see CompressorController and SENSOR_V_FAULT_* below).
"""

import json
import os
import signal
import sys
import threading
import time

from gpiozero import OutputDevice, Button
from smbus2 import SMBus
from luma.core.interface.serial import i2c as luma_i2c
from luma.core.render import canvas
from luma.oled.device import ssd1309
import serial
import pynmea2
from PIL import Image, ImageFont

# Uses PixelOperator.ttf (a free pixel font) for the compact preset list,
# if it's present next to this script. Falls back to PIL's built-in font
# if the file isn't there yet — drop PixelOperator.ttf in alongside this
# script to switch over, no code changes needed.
PIXEL_FONT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "PixelOperator.ttf")

# Pressure readings (corner numbers + F/R targets) use a bold monospace
# font instead — at this size PixelOperator's digits got compared side by
# side against several alternatives and came out one of the LEAST legible
# (its strokes are thin enough to blur together at small 1-bit sizes).
# DejaVu Sans Mono Bold ships by default on most Debian / Raspberry Pi OS
# installs (the `fonts-dejavu-core` package), so no extra file is needed —
# if it's somehow missing, this falls back to PixelOperator at a larger
# size, then to PIL's built-in font.
BIG_FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
]


def _load_font(size: int):
    try:
        return ImageFont.truetype(PIXEL_FONT_PATH, size)
    except Exception as exc:
        print(f"[display] could not load {PIXEL_FONT_PATH} ({exc}) — using built-in font instead")
        try:
            return ImageFont.load_default(size=size)
        except TypeError:
            return ImageFont.load_default()


def _load_big_font(size: int):
    for path in BIG_FONT_CANDIDATES:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    print("[display] no bold monospace font found (try: sudo apt install fonts-dejavu-core) "
          "— falling back to PixelOperator for pressure readings")
    return _load_font(size + 7)  # PixelOperator needs a larger size for similar visual height


SMALL_FONT = _load_font(16)        # preset list
BIG_FONT = _load_big_font(17)      # all pressure readings — corner numbers + F/R targets

# ============================================================
# Configuration
# ============================================================

# --- Which corners are physically wired up right now ---
ACTIVE_CORNERS = ["FL"]   # <-- add "FR", "RL", "RR" here as they're wired

CORNER_DISPLAY_NAME = {
    "FL": "FRONT LEFT", "FR": "FRONT RIGHT",
    "RL": "REAR LEFT", "RR": "REAR RIGHT",
}

# --- Compressor ---
COMPRESSOR_PIN = 4
COMPRESSOR_LINGER_SECONDS = 5.0   # keep running this long after inflate demand
                                   # ends, so it doesn't chatter on/off between
                                   # the individual inflate pulses
COMPRESSOR_POLL_SECONDS = 0.5      # how often the compressor thread re-checks demand

# --- Solenoid GPIO pins, per corner (all 4 defined; only ACTIVE_CORNERS used) ---
SOLENOID_PINS = {
    "FL": {"inflate": 17, "deflate": 27},
    "FR": {"inflate": 22, "deflate": 23},
    "RL": {"inflate": 24, "deflate": 25},
    "RR": {"inflate": 5,  "deflate": 6},
}

# --- Pressure sensors: ADS1115 channel per corner ---
SENSOR_CHANNELS = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}
ADS1115_ADDR = 0x48
ADS1115_BUS = 1

# --- Which axle each corner belongs to (front pair / rear pair) ---
CORNER_AXLE = {"FL": "front", "FR": "front", "RL": "rear", "RR": "rear"}

# --- Sensor calibration (0.5V=0psi, 4.5V=150psi, confirmed linear) ---
SENSOR_V_MIN = 0.5
SENSOR_V_MAX = 4.5
PSI_PER_VOLT = 150.0 / (SENSOR_V_MAX - SENSOR_V_MIN)  # 37.5

# A reading outside this (slightly widened) band means the sensor is
# disconnected/shorted, not a genuine pressure — treat as a fault and do NOT
# inflate toward a bogus "0psi". Margins allow for a little under/overshoot at
# the rails before flagging a fault.
SENSOR_V_FAULT_LOW = 0.25
SENSOR_V_FAULT_HIGH = 4.75
SENSOR_SAMPLES = 3   # samples per read; median used to reject noise

# --- Buttons ---
BUTTON_UP_PIN = 16
BUTTON_DOWN_PIN = 26
BUTTON_SELECT_PIN = 13
PSI_STEP = 1
MIN_PSI = 10
MAX_PSI = 60

# --- Inflate/deflate control timing ---
CHECK_INTERVAL_SECONDS = 3.0
SETTLE_SECONDS = 1.0
TOLERANCE_PSI = 1.0
WARNING_TRIGGER_SECONDS = 180.0   # continuous unsuccessful inflate -> diagnose & warn

# --- Presets: front/rear target psi, persisted to disk ---
PRESET_ORDER = ["ROAD", "DIRT", "SAND", "CUSTOM"]
PRESET_DISPLAY_NAME = {"ROAD": "ROAD", "DIRT": "DIRT", "SAND": "SAND", "CUSTOM": "CUST"}
STAGE1_ALLOWED_PRESETS = ["ROAD", "DIRT", "SAND"]   # CUSTOM excluded at 20-80km/h
DEFAULT_PRESETS = {
    "ROAD":   {"front": 36.0, "rear": 38.0},
    "DIRT":   {"front": 26.0, "rear": 28.0},
    "SAND":   {"front": 16.0, "rear": 18.0},
    "CUSTOM": {"front": 30.0, "rear": 30.0},
}
PRESET_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tyre_presets.json")
PRESET_SAVE_DEBOUNCE_SECONDS = 1.0

# --- GPS / speed lockout ---
GPS_PORT = "/dev/serial0"
GPS_BAUD = 9600
GPS_FIX_TIMEOUT_SECONDS = 5.0
SPEED_STAGE1_MIN_KMH = 20.0
SPEED_STAGE2_MIN_KMH = 80.0

# --- OLED display ---
OLED_WIDTH = 128
OLED_HEIGHT = 64
OLED_I2C_ADDRESS = 0x3C
DISPLAY_REFRESH_SECONDS = 0.5
WARNING_FLASH_SECONDS = 2.0   # 2s warning screen / 2s normal screen

# --- Startup splash screen (logo doesn't read clearly shrunk into the small
# vehicle-body slot, so it's shown full-screen for a few seconds at boot
# instead). Drop your logo image in next to this script as splash_logo.png
# (any size/aspect — it's auto-fitted to the 128x64 screen, centered, with
# white pixels treated as foreground).
SPLASH_LOGO_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "splash_logo.png")
SPLASH_DURATION_SECONDS = 3.0


class SensorFault(Exception):
    """Raised when a sensor reading is outside its valid voltage band
    (disconnected / shorted) — there's no trustworthy pressure to act on."""


# ============================================================
# Hardware: solenoid
# ============================================================
class AirSolenoid:
    def __init__(self, pin: int, active_high: bool = True):
        self._device = OutputDevice(pin, active_high=active_high, initial_value=False)
        self.pin = pin

    def open(self):
        self._device.on()

    def close(self):
        self._device.off()

    @property
    def is_open(self) -> bool:
        return self._device.value == 1

    def cleanup(self):
        self.close()
        self._device.close()


# ============================================================
# Hardware: compressor power switch (demand-based control)
# ============================================================
class CompressorController:
    """Powers the compressor only while inflation is actually needed.

    Turns on the moment demand appears; turns off COMPRESSOR_LINGER_SECONDS
    after demand last seen, so brief gaps between inflate pulses don't cause
    rapid on/off chatter. The compressor's own pressure-switch cutout remains
    the final mechanical backstop. Starts OFF."""

    def __init__(self, pin: int):
        self._device = OutputDevice(pin, active_high=True, initial_value=False)
        self._on = False
        self._last_demand = 0.0

    def request(self, demand: bool):
        now = time.monotonic()
        if demand:
            self._last_demand = now
            if not self._on:
                self._device.on()
                self._on = True
                print("[compressor] ON (inflate demand)")
        elif self._on and (now - self._last_demand) >= COMPRESSOR_LINGER_SECONDS:
            self._device.off()
            self._on = False
            print("[compressor] OFF (idle)")

    def cleanup(self):
        self._device.off()
        self._device.close()


def compressor_worker(controller: CompressorController, corners: dict,
                      stop_event: threading.Event):
    while not stop_event.is_set():
        demand = any(c.status == "INFLATE" for c in corners.values())
        controller.request(demand)
        _interruptible_sleep(COMPRESSOR_POLL_SECONDS, stop_event)


# ============================================================
# Hardware: ADS1115 (4-channel ADC) + pressure sensors
# ============================================================
class ADS1115:
    _REG_CONFIG = 0x01
    _REG_CONVERSION = 0x00
    _FULL_SCALE_VOLTS = 6.144  # PGA = 2/3x, covers our 0-4.5V signal directly

    def __init__(self, bus_num: int = ADS1115_BUS, address: int = ADS1115_ADDR):
        self._bus = SMBus(bus_num)
        self._address = address
        # All corner threads share one ADC. A read is a non-atomic
        # write-config -> wait -> read-conversion sequence, so it must be
        # serialised — otherwise two threads selecting different channels at
        # once would cross-wire their readings.
        self._lock = threading.Lock()

    def read_voltage(self, channel: int) -> float:
        if channel not in (0, 1, 2, 3):
            raise ValueError("channel must be 0-3")
        mux = (4 + channel) << 12
        config = 0x8183 | mux
        with self._lock:
            self._bus.write_i2c_block_data(
                self._address, self._REG_CONFIG, [(config >> 8) & 0xFF, config & 0xFF]
            )
            time.sleep(0.01)
            data = self._bus.read_i2c_block_data(self._address, self._REG_CONVERSION, 2)
        raw = (data[0] << 8) | data[1]
        if raw > 32767:
            raw -= 65536
        return raw * (self._FULL_SCALE_VOLTS / 32768.0)

    def close(self):
        self._bus.close()


class PressureSensor:
    def __init__(self, adc: ADS1115, channel: int):
        self._adc = adc
        self._channel = channel

    def read_psi(self) -> float:
        # Take several samples and use the median to reject electrical noise
        # around the tolerance band. Any single out-of-band sample means the
        # sensor is disconnected/shorted -> raise SensorFault rather than
        # report a bogus pressure.
        readings = []
        for _ in range(SENSOR_SAMPLES):
            v = self._adc.read_voltage(self._channel)
            if v < SENSOR_V_FAULT_LOW or v > SENSOR_V_FAULT_HIGH:
                raise SensorFault(
                    f"voltage {v:.3f}V outside valid {SENSOR_V_MIN}-{SENSOR_V_MAX}V band "
                    "(sensor disconnected or shorted?)"
                )
            readings.append(v)
        readings.sort()
        v = readings[len(readings) // 2]   # median
        psi = (v - SENSOR_V_MIN) * PSI_PER_VOLT
        return max(0.0, min(150.0, psi))


# ============================================================
# GPS + speed-based lockout
# ============================================================
class LockoutState:
    def __init__(self):
        self._lock = threading.Lock()
        self._stage = 2
        self._speed_kmh = 0.0
        self._gps_ok = False

    def update(self, speed_kmh, gps_ok: bool):
        with self._lock:
            self._gps_ok = gps_ok
            if not gps_ok or speed_kmh is None:
                self._speed_kmh = 0.0
                self._stage = 2
                return
            self._speed_kmh = speed_kmh
            if speed_kmh > SPEED_STAGE2_MIN_KMH:
                self._stage = 2
            elif speed_kmh >= SPEED_STAGE1_MIN_KMH:
                self._stage = 1
            else:
                self._stage = 0

    def stage(self) -> int:
        with self._lock:
            return self._stage

    def speed_kmh(self) -> float:
        with self._lock:
            return self._speed_kmh

    def gps_ok(self) -> bool:
        with self._lock:
            return self._gps_ok


def gps_worker(lockout: LockoutState, stop_event: threading.Event):
    last_fix_monotonic = 0.0
    try:
        ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
    except Exception as exc:
        print(f"[gps] could not open {GPS_PORT}: {exc} — staying in fail-safe lockout")
        while not stop_event.is_set():
            lockout.update(None, False)
            stop_event.wait(timeout=GPS_FIX_TIMEOUT_SECONDS)
        return

    while not stop_event.is_set():
        try:
            raw = ser.readline().decode("ascii", errors="replace").strip()
        except Exception as exc:
            print(f"[gps] read error: {exc}")
            raw = ""

        if raw.startswith("$") and "RMC" in raw:
            try:
                msg = pynmea2.parse(raw)
                if getattr(msg, "status", None) == "A" and msg.spd_over_grnd is not None:
                    speed_kmh = float(msg.spd_over_grnd) * 1.852
                    last_fix_monotonic = time.monotonic()
                    lockout.update(speed_kmh, True)
            except (pynmea2.ParseError, ValueError, AttributeError):
                pass

        if time.monotonic() - last_fix_monotonic > GPS_FIX_TIMEOUT_SECONDS:
            lockout.update(None, False)

    ser.close()


# ============================================================
# Preset manager: 4 presets x {front, rear} psi, persisted to disk
# ============================================================
class PresetManager:
    def __init__(self, path: str = PRESET_FILE):
        self._path = path
        self._lock = threading.Lock()
        self._presets = self._load()
        self._active = "ROAD"
        self._focus_axle = "front"
        self._save_timer = None

    def _load(self) -> dict:
        try:
            with open(self._path) as f:
                data = json.load(f)
            presets = {}
            for name in PRESET_ORDER:
                saved = data.get(name, {})
                presets[name] = {
                    "front": float(saved.get("front", DEFAULT_PRESETS[name]["front"])),
                    "rear": float(saved.get("rear", DEFAULT_PRESETS[name]["rear"])),
                }
            return presets
        except (FileNotFoundError, json.JSONDecodeError, ValueError, OSError):
            return {name: dict(vals) for name, vals in DEFAULT_PRESETS.items()}

    def _schedule_save(self):
        if self._save_timer:
            self._save_timer.cancel()
        self._save_timer = threading.Timer(PRESET_SAVE_DEBOUNCE_SECONDS, self._save)
        self._save_timer.daemon = True
        self._save_timer.start()

    def _save(self):
        with self._lock:
            snapshot = json.loads(json.dumps(self._presets))
        try:
            with open(self._path, "w") as f:
                json.dump(snapshot, f, indent=2)
            print(f"[presets] saved to {self._path}")
        except OSError as exc:
            print(f"[presets] save failed: {exc}")

    def active_name(self) -> str:
        with self._lock:
            return self._active

    def focus_axle(self) -> str:
        with self._lock:
            return self._focus_axle

    def get_target(self, axle: str) -> float:
        with self._lock:
            return self._presets[self._active][axle]

    def get_targets(self):
        with self._lock:
            p = self._presets[self._active]
            return p["front"], p["rear"]

    def cycle_preset(self, allowed=None):
        with self._lock:
            order = allowed if allowed else PRESET_ORDER
            if self._active in order:
                idx = order.index(self._active)
                self._active = order[(idx + 1) % len(order)]
            else:
                self._active = order[0]
            print(f"[presets] active preset -> {self._active}")

    def enforce_allowed(self, allowed):
        with self._lock:
            if allowed and self._active not in allowed:
                old = self._active
                self._active = allowed[0]
                print(f"[presets] speed lockout: switched from {old} to {self._active}")

    def toggle_focus_axle(self):
        with self._lock:
            self._focus_axle = "rear" if self._focus_axle == "front" else "front"
            print(f"[presets] editing axle -> {self._focus_axle}")

    def adjust(self, delta: float):
        with self._lock:
            axle = self._focus_axle
            val = self._presets[self._active][axle] + delta
            val = max(MIN_PSI, min(MAX_PSI, val))
            self._presets[self._active][axle] = val
            print(f"[presets] {self._active}.{axle} -> {val:.0f} psi")
        self._schedule_save()


# ============================================================
# Warning manager: punctures (per corner) + possible compressor failure
# ============================================================
class WarningManager:
    """Tracks suspected punctures (per corner) and a system-wide possible
    compressor-failure flag. Each warning has an 'active' state (the
    underlying condition is currently true) and a 'paused' state (the
    flashing has been silenced by a button press, but the condition
    hasn't necessarily resolved)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._punctures = {}                                  # name -> {"active","paused"}
        self._compressor_failure = {"active": False, "paused": False}

    def set_puncture(self, name: str, active: bool):
        with self._lock:
            existing = self._punctures.get(name, {"active": False, "paused": False})
            if active and not existing["active"]:
                print(f"[warning] PUNCTURE suspected: {name}")
                self._punctures[name] = {"active": True, "paused": False}
            elif not active and existing["active"]:
                print(f"[warning] {name} puncture warning cleared")
                self._punctures[name] = {"active": False, "paused": False}
            elif active:
                existing["active"] = True  # re-confirm, keep existing paused state

    def has_puncture(self, name: str) -> bool:
        with self._lock:
            entry = self._punctures.get(name)
            return bool(entry and entry["active"])

    def set_compressor_failure(self, active: bool):
        with self._lock:
            if active and not self._compressor_failure["active"]:
                print("[warning] POSSIBLE COMPRESSOR FAILURE suspected")
                self._compressor_failure = {"active": True, "paused": False}
            elif not active and self._compressor_failure["active"]:
                print("[warning] compressor failure warning cleared")
                self._compressor_failure = {"active": False, "paused": False}

    def maybe_clear_compressor_failure(self, corner_names):
        with self._lock:
            if self._compressor_failure["active"]:
                if any(not self._punctures.get(n, {}).get("active", False) for n in corner_names):
                    self._compressor_failure = {"active": False, "paused": False}
                    print("[warning] compressor failure warning cleared (air is flowing somewhere)")

    def pause_all(self) -> bool:
        with self._lock:
            paused_any = False
            for entry in self._punctures.values():
                if entry["active"] and not entry["paused"]:
                    entry["paused"] = True
                    paused_any = True
            if self._compressor_failure["active"] and not self._compressor_failure["paused"]:
                self._compressor_failure["paused"] = True
                paused_any = True
            return paused_any

    def get_unpaused_active(self):
        """List of (kind, name_or_None) for warnings currently flashing."""
        with self._lock:
            items = [("PUNCTURE", n) for n, e in self._punctures.items() if e["active"] and not e["paused"]]
            if self._compressor_failure["active"] and not self._compressor_failure["paused"]:
                items.append(("COMPRESSOR_FAILURE", None))
            return items

    def has_unpaused_active(self) -> bool:
        return bool(self.get_unpaused_active())

    def get_paused_highlights(self):
        """(set of corner names to show inverted, compressor_failure_paused: bool)"""
        with self._lock:
            names = {n for n, e in self._punctures.items() if e["active"] and e["paused"]}
            comp = self._compressor_failure["active"] and self._compressor_failure["paused"]
            return names, comp


# ============================================================
# One tyre corner: sensor + inflate/deflate solenoids + live state
# ============================================================
class TyreCorner:
    def __init__(self, name: str, axle: str, sensor: PressureSensor,
                 inflate: AirSolenoid, deflate: AirSolenoid):
        self.name = name
        self.axle = axle
        self.sensor = sensor
        self.inflate = inflate
        self.deflate = deflate
        self.actual_psi = 0.0
        self.status = "--"     # "OK", "INFLATE", "DEFLATE", "FAULT"
        self.fault = False     # sensor read failure / out-of-range (no trustworthy data)

    def cleanup(self):
        self.inflate.cleanup()
        self.deflate.cleanup()


def _interruptible_sleep(duration: float, stop_event: threading.Event):
    stop_event.wait(timeout=duration)


def corner_worker(corner: TyreCorner, corners: dict, presets: PresetManager,
                   warnings: WarningManager, stop_event: threading.Event):
    """Runs continuously for one tyre. Speed-based lockout (see
    LockoutState) only ever gates the buttons/UI — it does NOT pause
    this loop. Pressure is checked and inflate/deflate keeps running
    at every speed, including above the Stage-2 threshold."""
    run_start = None  # when this corner started continuously trying to inflate

    while not stop_event.is_set():
        try:
            corner.actual_psi = corner.sensor.read_psi()
        except Exception as exc:
            # No trustworthy pressure data (I2C error or out-of-range voltage):
            # close solenoids and wait. Self-healing — the next valid reading
            # clears the fault automatically (no permanent latch).
            if not corner.fault:
                print(f"[{corner.name}] sensor fault: {exc}")
            corner.status = "FAULT"
            corner.fault = True
            corner.inflate.close()
            corner.deflate.close()
            run_start = None
            _interruptible_sleep(CHECK_INTERVAL_SECONDS, stop_event)
            continue

        if corner.fault:
            print(f"[{corner.name}] sensor recovered — resuming control")
            corner.fault = False

        target = presets.get_target(corner.axle)
        diff = target - corner.actual_psi

        if diff > TOLERANCE_PSI:
            corner.status = "INFLATE"
            run_start = run_start or time.monotonic()
            corner.deflate.close()
            corner.inflate.open()
            _interruptible_sleep(CHECK_INTERVAL_SECONDS, stop_event)
            corner.inflate.close()
            _interruptible_sleep(SETTLE_SECONDS, stop_event)

        elif diff < -TOLERANCE_PSI:
            corner.status = "DEFLATE"
            run_start = None
            corner.inflate.close()
            corner.deflate.open()
            _interruptible_sleep(CHECK_INTERVAL_SECONDS, stop_event)
            corner.deflate.close()
            _interruptible_sleep(SETTLE_SECONDS, stop_event)

        else:
            corner.status = "OK"
            run_start = None
            corner.inflate.close()
            corner.deflate.close()
            warnings.set_puncture(corner.name, False)
            warnings.maybe_clear_compressor_failure(corners.keys())
            _interruptible_sleep(CHECK_INTERVAL_SECONDS, stop_event)

        # Struggling to inflate for too long? Diagnose, warn, but KEEP TRYING —
        # no permanent lock for this case, just a flag and a re-check later.
        if run_start and (time.monotonic() - run_start) > WARNING_TRIGGER_SECONDS:
            other_axle_names = [n for n, c in corners.items() if c.axle != corner.axle]
            other_axle_healthy = any(not warnings.has_puncture(n) for n in other_axle_names)
            if other_axle_names and other_axle_healthy:
                warnings.set_puncture(corner.name, True)
            else:
                warnings.set_compressor_failure(True)
            run_start = time.monotonic()  # give it a fresh window before re-diagnosing


# ============================================================
# OLED display: header + 4WD outline + preset bar + warning flashing
# ============================================================
def load_splash_logo():
    """Loads the startup splash image, fitted to the full 128x64 screen
    and centered on a black canvas. Returns a PIL Image in mode "1", or
    None if no logo file is present / it failed to load — startup just
    skips the splash in that case."""
    if not os.path.exists(SPLASH_LOGO_PATH):
        print(f"[display] no splash logo at {SPLASH_LOGO_PATH} — skipping splash screen")
        return None
    try:
        img = Image.open(SPLASH_LOGO_PATH).convert("1")
        img.thumbnail((OLED_WIDTH, OLED_HEIGHT))
        canvas_img = Image.new("1", (OLED_WIDTH, OLED_HEIGHT), 0)
        x = (OLED_WIDTH - img.width) // 2
        y = (OLED_HEIGHT - img.height) // 2
        canvas_img.paste(img, (x, y))
        return canvas_img
    except Exception as exc:
        print(f"[display] could not load splash logo ({exc}) — skipping splash screen")
        return None


def show_splash(device, splash_logo, duration: float = SPLASH_DURATION_SECONDS):
    """Shows the full-screen startup logo for `duration` seconds. No-op
    if no splash logo was loaded."""
    if splash_logo is None:
        return
    with canvas(device) as draw:
        draw.bitmap((0, 0), splash_logo, fill="white")
    time.sleep(duration)


def _draw_text_block(draw, x, y, lines, line_height, highlight: bool, font=None):
    """Draws a small block of text lines, optionally as an inverted
    (white box, black text) highlight. Box size is measured from the
    actual font metrics rather than guessed from character count."""
    font = font or SMALL_FONT
    if highlight:
        max_w = max(draw.textbbox((0, 0), line, font=font)[2] for line in lines)
        height = line_height * len(lines) + 2
        draw.rectangle((x - 1, y - 1, x - 1 + max_w + 2, y - 1 + height), fill="white")
        color = "black"
    else:
        color = "white"
    for i, line in enumerate(lines):
        draw.text((x, y + i * line_height), line, fill=color, font=font)


CORNER_FLASH_INTERVAL_SECONDS = 0.6  # how often an active corner's reading
                                      # alternates between the number and an
                                      # up/down arrow while inflating/deflating

CORNER_GAP = 1          # fixed gap between the wheel edge and its reading —
                         # same gap on every corner regardless of digit count
TRIANGLE_SIZE = 7       # half-extent of the inflate/deflate arrow triangle


def _draw_corner_reading(draw, x_ref: int, cy: int, anchor: str, corner, highlighted: bool):
    """Draws one corner's pressure reading: the bare number normally, or
    alternating number/arrow (triangle) while actively inflating or
    deflating. No FL/FR/RL/RR label — position alone identifies it.

    anchor="right": content's right edge is pinned to x_ref (for readings
        sitting to the LEFT of a wheel — FL/RL).
    anchor="left":  content's left edge is pinned to x_ref (readings to
        the RIGHT of a wheel — FR/RR).
    Vertically centered on cy. Content width is measured from the actual
    text/triangle, so the gap to the wheel is identical on every corner —
    no fixed box that wide digits would crowd and narrow ones would float
    away from.
    """
    fg = "black" if highlighted else "white"
    is_triangle = False

    if corner is None:
        text = "--"
    elif corner.status == "FAULT":
        text = "ERR"
    else:
        flashing = corner.status in ("INFLATE", "DEFLATE")
        show_arrow = flashing and (int(time.monotonic() // CORNER_FLASH_INTERVAL_SECONDS) % 2 == 1)
        if show_arrow:
            is_triangle = True
            triangle_up = corner.status == "INFLATE"
        else:
            text = f"{corner.actual_psi:.0f}"

    if is_triangle:
        w = h = TRIANGLE_SIZE * 2
        top = 0
    else:
        l, top, r, b = draw.textbbox((0, 0), text, font=BIG_FONT)
        w, h = r - l, b - top

    extra = 1 if is_triangle else 0   # nudge arrows 1px further from the wheel than the number
    x = (x_ref - extra - w) if anchor == "right" else (x_ref + extra)
    ink_top = cy - h // 2

    if highlighted:
        draw.rectangle((x - 2, ink_top - 2, x + w + 2, ink_top + h + 2), fill="white")

    if is_triangle:
        bx, by = x + w // 2, ink_top + h // 2
        s = TRIANGLE_SIZE
        if triangle_up:
            pts = [(bx, by - s), (bx - s, by + s), (bx + s, by + s)]
        else:
            pts = [(bx, by + s), (bx - s, by - s), (bx + s, by - s)]
        draw.polygon(pts, fill=fg)
    else:
        draw.text((x, ink_top - top), text, fill=fg, font=BIG_FONT)


def render_dashboard(draw, corners: dict, presets: PresetManager, warnings: WarningManager):
    W, H = OLED_WIDTH, OLED_HEIGHT
    paused_names, comp_paused = warnings.get_paused_highlights()

    # --- Axle diagram: wheel — bar — diff-housing square — bar — wheel,
    # front and rear housings linked by a vertical driveshaft line.
    # Large, left-aligned, nearly full screen height. Geometry below is
    # solved so corner readings + F/R labels (both at the larger font)
    # and the preset stack all fit the 128px width with no overlap.
    cx = 47
    wheel_w, wheel_h = 6, 18    # narrow tyres
    square = 6                   # diff housing: small box only, no nested detail
    bar_gap = 14                 # wide enough for the bigger F/R label to sit between

    front_wheel_top, front_wheel_bottom = 2, 2 + wheel_h
    rear_wheel_bottom, rear_wheel_top = H - 2, H - 2 - wheel_h
    front_cy = (front_wheel_top + front_wheel_bottom) // 2
    rear_cy = (rear_wheel_top + rear_wheel_bottom) // 2

    front_square = (cx - square // 2, front_cy - square // 2, cx + square // 2, front_cy + square // 2)
    rear_square = (cx - square // 2, rear_cy - square // 2, cx + square // 2, rear_cy + square // 2)

    wheel_boxes = {
        "FL": (cx - square // 2 - bar_gap - wheel_w, front_wheel_top, cx - square // 2 - bar_gap, front_wheel_bottom),
        "FR": (cx + square // 2 + bar_gap, front_wheel_top, cx + square // 2 + bar_gap + wheel_w, front_wheel_bottom),
        "RL": (cx - square // 2 - bar_gap - wheel_w, rear_wheel_top, cx - square // 2 - bar_gap, rear_wheel_bottom),
        "RR": (cx + square // 2 + bar_gap, rear_wheel_top, cx + square // 2 + bar_gap + wheel_w, rear_wheel_bottom),
    }
    for box in wheel_boxes.values():
        draw.rounded_rectangle(box, radius=2, fill="white")

    # Connecting bars: wheel edge -> housing edge, at that axle's centerline
    draw.line((wheel_boxes["FL"][2], front_cy, front_square[0], front_cy), fill="white")
    draw.line((front_square[2], front_cy, wheel_boxes["FR"][0], front_cy), fill="white")
    draw.line((wheel_boxes["RL"][2], rear_cy, rear_square[0], rear_cy), fill="white")
    draw.line((rear_square[2], rear_cy, wheel_boxes["RR"][0], rear_cy), fill="white")

    # F/R target labels, sitting between the housings (computed first so
    # the driveshaft line below can stop short of them, rather than
    # running straight through the text)
    front_t, rear_t = presets.get_targets()
    f_label = f"F{front_t:.0f}"
    r_label = f"R{rear_t:.0f}"
    fl, ft, fr, fb = draw.textbbox((0, 0), f_label, font=BIG_FONT)
    rl, rt, rr, rb = draw.textbbox((0, 0), r_label, font=BIG_FONT)
    f_w, f_h = fr - fl, fb - ft
    r_w, r_h = rr - rl, rb - rt
    label_gap = 3   # gap from each housing/line-end to its label
    f_ink_top = front_square[3] + label_gap
    r_ink_top = rear_square[1] - label_gap - r_h + 1

    # Driveshaft: front housing -> top of F label, and bottom of R label -> rear housing
    draw.line((cx, front_square[3], cx, f_ink_top - 2), fill="white")
    draw.line((cx, r_ink_top + r_h + 2, cx, rear_square[1]), fill="white")

    # Diff housings: a small outlined box only — no nested inner detail
    draw.rectangle(front_square, outline="white")
    draw.rectangle(rear_square, outline="white")

    _draw_text_block(draw, cx - f_w // 2, f_ink_top - ft, [f_label], f_h + 2, highlight=comp_paused, font=BIG_FONT)
    _draw_text_block(draw, cx - r_w // 2, r_ink_top - rt, [r_label], r_h + 2, highlight=comp_paused, font=BIG_FONT)

    # --- Corner pressure readings, just outside each wheel — same fixed
    # gap (CORNER_GAP) from the wheel edge on every corner ---
    corner_specs = {
        "FL": (wheel_boxes["FL"][0] - CORNER_GAP, front_cy, "right"),
        "FR": (wheel_boxes["FR"][2] + CORNER_GAP, front_cy, "left"),
        "RL": (wheel_boxes["RL"][0] - CORNER_GAP, rear_cy, "right"),
        "RR": (wheel_boxes["RR"][2] + CORNER_GAP, rear_cy, "left"),
    }
    for name, (x_ref, cy, anchor) in corner_specs.items():
        _draw_corner_reading(draw, x_ref, cy, anchor, corners.get(name), highlighted=(name in paused_names))

    # --- Right-hand side: stacked preset names, active one highlighted ---
    active = presets.active_name()
    px = W - 34
    for i, name in enumerate(PRESET_ORDER):
        _draw_text_block(draw, px, 2 + i * 15, [PRESET_DISPLAY_NAME[name]], 13,
                          highlight=(name == active))


def render_warning_screen(draw, kind: str, name):
    """Full white screen, black text — used while a warning is flashing."""
    W, H = OLED_WIDTH, OLED_HEIGHT
    draw.rectangle((0, 0, W, H), fill="white")

    if kind == "PUNCTURE":
        lines = ["**WARNING**", f"{CORNER_DISPLAY_NAME[name]} TYRE", "PUNCTURE"]
    else:
        lines = ["**WARNING**", "POSSIBLE COMPRESSOR", "FAILURE"]

    y = 2
    for line in lines:
        draw.text((4, y), line, fill="black", font=SMALL_FONT)
        y += 10

    draw.text((4, H - 20), "Push any button to", fill="black", font=SMALL_FONT)
    draw.text((4, H - 10), "pause warning", fill="black", font=SMALL_FONT)


class FlashState:
    def __init__(self):
        self.phase = "normal"      # "normal" or "warning"
        self.phase_start = time.monotonic()
        self.queue_index = 0


def display_loop(device, corners: dict, presets: PresetManager,
                  lockout: LockoutState, warnings: WarningManager,
                  stop_event: threading.Event):
    flash = FlashState()

    while not stop_event.is_set():
        stage = lockout.stage()
        if stage >= 1:
            presets.enforce_allowed(STAGE1_ALLOWED_PRESETS)

        active_warnings = warnings.get_unpaused_active()
        now = time.monotonic()

        if not active_warnings:
            flash.phase = "normal"
            with canvas(device) as draw:
                render_dashboard(draw, corners, presets, warnings)
        else:
            elapsed = now - flash.phase_start
            if flash.phase == "normal" and elapsed >= WARNING_FLASH_SECONDS:
                flash.phase = "warning"
                flash.phase_start = now
            elif flash.phase == "warning" and elapsed >= WARNING_FLASH_SECONDS:
                flash.phase = "normal"
                flash.phase_start = now
                flash.queue_index = (flash.queue_index + 1) % len(active_warnings)

            if flash.phase == "warning":
                kind, name = active_warnings[flash.queue_index % len(active_warnings)]
                with canvas(device) as draw:
                    render_warning_screen(draw, kind, name)
            else:
                with canvas(device) as draw:
                    render_dashboard(draw, corners, presets, warnings)

        _interruptible_sleep(DISPLAY_REFRESH_SECONDS, stop_event)


# ============================================================
# Main
# ============================================================
def main():
    stop_event = threading.Event()

    # Demand-based: starts OFF, the compressor thread powers it only when a
    # corner actually needs to inflate.
    compressor = CompressorController(COMPRESSOR_PIN)
    print("[compressor] idle (demand-based control)")

    adc = ADS1115()
    sensors = {name: PressureSensor(adc, SENSOR_CHANNELS[name]) for name in ACTIVE_CORNERS}

    corners = {}
    for name in ACTIVE_CORNERS:
        pins = SOLENOID_PINS[name]
        corners[name] = TyreCorner(
            name=name,
            axle=CORNER_AXLE[name],
            sensor=sensors[name],
            inflate=AirSolenoid(pins["inflate"]),
            deflate=AirSolenoid(pins["deflate"]),
        )

    presets = PresetManager()
    lockout = LockoutState()
    warnings = WarningManager()

    btn_up = Button(BUTTON_UP_PIN, pull_up=True, bounce_time=0.05)
    btn_down = Button(BUTTON_DOWN_PIN, pull_up=True, bounce_time=0.05)
    btn_select = Button(BUTTON_SELECT_PIN, pull_up=True, bounce_time=0.05)

    def on_up():
        if warnings.has_unpaused_active():
            warnings.pause_all()
            return
        if lockout.stage() >= 2:
            print("[lockout] UP ignored — Stage 2 (speed lockout)")
            return
        presets.adjust(PSI_STEP)

    def on_down():
        if warnings.has_unpaused_active():
            warnings.pause_all()
            return
        if lockout.stage() >= 2:
            print("[lockout] DOWN ignored — Stage 2 (speed lockout)")
            return
        presets.adjust(-PSI_STEP)

    btn_up.when_pressed = on_up
    btn_up.hold_time = 0.4
    btn_up.hold_repeat = True
    btn_up.when_held = on_up

    btn_down.when_pressed = on_down
    btn_down.hold_time = 0.4
    btn_down.hold_repeat = True
    btn_down.when_held = on_down

    select_state = {"held": False}
    btn_select.hold_time = 1.0
    btn_select.hold_repeat = False

    def on_select_held():
        select_state["held"] = True
        if warnings.has_unpaused_active():
            warnings.pause_all()
            return
        if lockout.stage() >= 2:
            print("[lockout] SELECT(hold) ignored — Stage 2 (speed lockout)")
            return
        presets.toggle_focus_axle()

    def on_select_released():
        held = select_state["held"]
        select_state["held"] = False
        if held:
            return  # already handled in on_select_held
        if warnings.has_unpaused_active():
            warnings.pause_all()
            return
        stage = lockout.stage()
        if stage >= 2:
            print("[lockout] SELECT ignored — Stage 2 (speed lockout)")
        else:
            allowed = STAGE1_ALLOWED_PRESETS if stage == 1 else None
            presets.cycle_preset(allowed=allowed)

    btn_select.when_held = on_select_held
    btn_select.when_released = on_select_released

    serial_iface = luma_i2c(port=1, address=OLED_I2C_ADDRESS)
    device = ssd1309(serial_iface, width=OLED_WIDTH, height=OLED_HEIGHT)
    show_splash(device, load_splash_logo())

    threads = [
        threading.Thread(target=corner_worker,
                          args=(corners[n], corners, presets, warnings, stop_event),
                          daemon=True)
        for n in corners
    ]
    threads.append(threading.Thread(
        target=display_loop, args=(device, corners, presets, lockout, warnings, stop_event), daemon=True))
    threads.append(threading.Thread(target=gps_worker, args=(lockout, stop_event), daemon=True))
    threads.append(threading.Thread(
        target=compressor_worker, args=(compressor, corners, stop_event), daemon=True))

    def shutdown(signum=None, frame=None):
        print("\n[system] Shutting down — closing all solenoids, stopping compressor...")
        stop_event.set()
        time.sleep(0.2)
        for c in corners.values():
            c.cleanup()
        compressor.cleanup()
        adc.close()
        device.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    for t in threads:
        t.start()

    print("[system] Running. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        shutdown()


if __name__ == "__main__":
    main()
