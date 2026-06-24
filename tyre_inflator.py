#!/usr/bin/env python3
"""
4WD Tyre Pressure Monitor & Auto-Inflator — Raspberry Pi 5

Full system overview, wiring, lockout/warning behaviour and setup notes
live in OVERVIEW.txt next to this script. Quick summary:

- Per-corner pressure monitoring via an ADS1115 ADC (0.5-4.5V = 0-150psi),
  with median sampling, out-of-band fault detection and I2C retries.
- Automatic inflate/deflate to ROAD/DIRT/SAND/CUSTOM presets (front/rear
  targets, persisted atomically to tyre_presets.json).
- Absolute over-pressure guard (force-deflate above HARD_MAX_PSI) plus
  inflate-struggle (puncture / compressor) and deflate-struggle warnings.
- SSD1309 OLED dashboard with a 4WD outline + flashing warnings.
- GPS speed-based 2-stage control lockout (UI only — regulation never
  stops; fail-safe to the most restrictive stage with no fix), with serial
  auto-reconnect.
- Tank-pressure compressor control (hysteresis), fully decoupled from the
  corner solenoids, plus self-healing sensor fault detection. The corner
  solenoids draw from a charged air tank on demand.

Relay polarity: many relay/MOSFET boards are ACTIVE-LOW. Set
SOLENOID_ACTIVE_HIGH / COMPRESSOR_ACTIVE_HIGH to match your hardware and
bench-verify the de-energised state before connecting air.
"""

import json
import logging
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

log = logging.getLogger("cti")

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
        log.warning(f"[display] could not load {PIXEL_FONT_PATH} ({exc}) — using built-in font instead")
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
    log.warning("[display] no bold monospace font found (try: sudo apt install fonts-dejavu-core) "
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

# --- Relay/MOSFET driver polarity ---
# True  = board energises the load on a HIGH signal (active-high).
# False = board energises on a LOW signal (active-low — common on cheap
#         opto-isolated relay boards). Either way, "off/closed" is the
#         de-energised state at startup; set this to match your board and
#         CONFIRM the de-energised state on the bench before piping air.
SOLENOID_ACTIVE_HIGH = True
COMPRESSOR_ACTIVE_HIGH = True

# --- Compressor + air tank ---
# The compressor and the corner solenoids are DECOUPLED. The compressor's only
# job is to keep the air TANK charged, controlled by a tank pressure sensor with
# hysteresis (on at CUT_IN, off at CUT_OUT). The corner solenoids draw from the
# tank on demand, independently. Keep a mechanical pressure switch wired as a
# hardware backstop, set just above TANK_PRESSURE_CUT_OUT, in case the sensor or
# software ever fails high.
COMPRESSOR_PIN = 4
TANK_PRESSURE_CUT_IN = 90.0      # turn compressor ON at/below this tank psi
TANK_PRESSURE_CUT_OUT = 120.0    # turn compressor OFF at/above this tank psi
TANK_SUPPLY_MIN_PSI = 50.0       # below this the tank can't reliably feed an air-up
TANK_FILL_TIMEOUT_SECONDS = 120.0  # compressor running this long without reaching
                                    # CUT_OUT -> suspect compressor failure / big leak
TANK_POLL_SECONDS = 1.0

# Tank pressure sensor location on the ADC. For bench testing with one ADS1115
# (corners on A0-A3, but only FL wired) the tank can share that board on a free
# channel. Once all four corners occupy 0x48 A0-A3, move the tank sensor to a
# SECOND ADS1115 (ADDR pin -> VDD = address 0x49) and update TANK_SENSOR_ADDR.
# Assumes the same 0.5-4.5V = 0-150psi calibration as the corner sensors; adjust
# the SENSOR_* constants if your tank sensor's range differs.
TANK_SENSOR_ADDR = 0x48
TANK_SENSOR_CHANNEL = 1

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
ADS1115_READ_ATTEMPTS = 3   # retry a transient I2C glitch before faulting the corner

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
WARNING_TRIGGER_SECONDS = 180.0   # continuous unsuccessful inflate/deflate -> diagnose & warn

# Absolute over-pressure ceiling, INDEPENDENT of the selected preset target.
# If a tyre ever exceeds this (runaway compressor pressure-switch, corrupted
# target, etc.) the corner force-deflates and warns, regardless of its target.
# Keep comfortably above MAX_PSI so normal regulation never trips it.
HARD_MAX_PSI = 65.0

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
GPS_MAX_READ_ERRORS = 5   # consecutive read errors before reopening the serial port
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


def _interruptible_sleep(duration: float, stop_event: threading.Event):
    stop_event.wait(timeout=duration)


# ============================================================
# Hardware: solenoid
# ============================================================
class AirSolenoid:
    def __init__(self, pin: int, active_high: bool = SOLENOID_ACTIVE_HIGH):
        # initial_value=False is always the de-energised (closed) state,
        # regardless of active_high — gpiozero maps the logical level to the
        # correct electrical level for the board polarity.
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
# Hardware: compressor + air tank (tank-pressure controlled)
# ============================================================
class TankController:
    """Keeps the air tank charged from a tank pressure sensor using simple
    hysteresis, decoupled from the corner solenoids: compressor ON at/below
    TANK_PRESSURE_CUT_IN, OFF at/above TANK_PRESSURE_CUT_OUT. Starts OFF.

    Raises the system-wide compressor-failure warning if the tank can't reach
    CUT_OUT within TANK_FILL_TIMEOUT_SECONDS (failing pump / big leak), and on
    a tank-sensor fault stops the pump as a fail-safe. A mechanical pressure
    switch should still be wired as the hardware backstop."""

    def __init__(self, pin: int, sensor: "PressureSensor",
                 active_high: bool = COMPRESSOR_ACTIVE_HIGH):
        self._device = OutputDevice(pin, active_high=active_high, initial_value=False)
        self._sensor = sensor
        self._on = False
        self._fill_start = None
        self.tank_psi = 0.0
        self.fault = False

    def _set(self, on: bool, reason: str):
        if on != self._on:
            (self._device.on if on else self._device.off)()
            self._on = on
            log.info(f"[compressor] {'ON' if on else 'OFF'} ({reason})")

    def update(self, warnings: "WarningManager"):
        try:
            self.tank_psi = self._sensor.read_psi()
            self.fault = False
        except Exception as exc:
            if not self.fault:
                log.warning(f"[tank] sensor fault: {exc} — stopping compressor (fail-safe)")
            self.fault = True
            self._set(False, "tank sensor fault")
            self._fill_start = None
            warnings.set_compressor_failure(True)
            return

        if not self._on and self.tank_psi <= TANK_PRESSURE_CUT_IN:
            self._set(True, f"tank {self.tank_psi:.0f} <= {TANK_PRESSURE_CUT_IN:.0f}")
            self._fill_start = time.monotonic()
        elif self._on and self.tank_psi >= TANK_PRESSURE_CUT_OUT:
            self._set(False, f"tank {self.tank_psi:.0f} >= {TANK_PRESSURE_CUT_OUT:.0f}")
            self._fill_start = None
            warnings.set_compressor_failure(False)

        # Running a long time without reaching cut-out -> failing pump / leak.
        if (self._on and self._fill_start
                and (time.monotonic() - self._fill_start) > TANK_FILL_TIMEOUT_SECONDS):
            warnings.set_compressor_failure(True)

    def tank_ok(self) -> bool:
        """True when the tank can currently supply an air-up (sensor healthy
        and above the usable-supply floor)."""
        return (not self.fault) and self.tank_psi >= TANK_SUPPLY_MIN_PSI

    def cleanup(self):
        self._device.off()
        self._device.close()


def tank_worker(controller: "TankController", warnings: "WarningManager",
                stop_event: threading.Event):
    while not stop_event.is_set():
        controller.update(warnings)
        _interruptible_sleep(TANK_POLL_SECONDS, stop_event)


# ============================================================
# Hardware: ADS1115 (4-channel ADC) + pressure sensors
# ============================================================
class ADS1115:
    _REG_CONFIG = 0x01
    _REG_CONVERSION = 0x00
    _FULL_SCALE_VOLTS = 6.144  # PGA = 2/3x, covers our 0-4.5V signal directly
    _CONVERSION_TIMEOUT = 0.05  # seconds to wait for a single-shot conversion

    def __init__(self, bus_num: int = ADS1115_BUS, address: int = ADS1115_ADDR):
        self._bus = SMBus(bus_num)
        self._address = address
        # All corner threads share one ADC. A read is a non-atomic
        # write-config -> wait -> read-conversion sequence, so it must be
        # serialised — otherwise two threads selecting different channels at
        # once would cross-wire their readings.
        self._lock = threading.Lock()

    def _wait_conversion_ready(self):
        # Poll the config register's OS bit (bit 15) rather than guessing a
        # fixed delay, so changing the data-rate bits can't silently under-wait.
        deadline = time.monotonic() + self._CONVERSION_TIMEOUT
        while time.monotonic() < deadline:
            cfg = self._bus.read_i2c_block_data(self._address, self._REG_CONFIG, 2)
            if cfg[0] & 0x80:   # OS=1 -> conversion complete
                return
            time.sleep(0.001)
        # Timed out — fall through and read whatever's there rather than hang.

    def read_voltage(self, channel: int) -> float:
        if channel not in (0, 1, 2, 3):
            raise ValueError("channel must be 0-3")
        mux = (4 + channel) << 12
        config = 0x8183 | mux
        hi, lo = (config >> 8) & 0xFF, config & 0xFF
        last_exc = None
        for _ in range(ADS1115_READ_ATTEMPTS):
            try:
                with self._lock:
                    self._bus.write_i2c_block_data(self._address, self._REG_CONFIG, [hi, lo])
                    self._wait_conversion_ready()
                    data = self._bus.read_i2c_block_data(self._address, self._REG_CONVERSION, 2)
                raw = (data[0] << 8) | data[1]
                if raw > 32767:
                    raw -= 65536
                return raw * (self._FULL_SCALE_VOLTS / 32768.0)
            except OSError as exc:
                last_exc = exc
                time.sleep(0.002)   # brief backoff before retrying the bus
        raise last_exc

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
    ser = None
    consecutive_errors = 0

    while not stop_event.is_set():
        # (Re)open the serial port as needed — recovers from an unplugged /
        # browned-out GPS instead of getting stuck in fail-safe forever.
        if ser is None:
            try:
                ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
                consecutive_errors = 0
                log.info(f"[gps] opened {GPS_PORT}")
            except Exception as exc:
                log.warning(f"[gps] could not open {GPS_PORT}: {exc} — staying in fail-safe lockout")
                lockout.update(None, False)
                stop_event.wait(timeout=GPS_FIX_TIMEOUT_SECONDS)
                continue

        try:
            raw = ser.readline().decode("ascii", errors="replace").strip()
            consecutive_errors = 0
        except Exception as exc:
            consecutive_errors += 1
            log.warning(f"[gps] read error: {exc}")
            raw = ""
            if consecutive_errors >= GPS_MAX_READ_ERRORS:
                log.warning("[gps] too many read errors — reopening serial port")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                lockout.update(None, False)
                stop_event.wait(timeout=1.0)
                continue

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

    if ser is not None:
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
        # Write to a temp file then atomically replace, so a power loss
        # mid-write can't corrupt tyre_presets.json (worst case the temp file
        # is left behind; the real file is always whole).
        tmp = self._path + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(snapshot, f, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self._path)
            log.info(f"[presets] saved to {self._path}")
        except OSError as exc:
            log.error(f"[presets] save failed: {exc}")

    def flush(self):
        """Cancel any pending debounced save and write immediately — used on
        shutdown so a tweak made right before power-off isn't lost."""
        if self._save_timer:
            self._save_timer.cancel()
            self._save_timer = None
        self._save()

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
            log.info(f"[presets] active preset -> {self._active}")

    def enforce_allowed(self, allowed):
        with self._lock:
            if allowed and self._active not in allowed:
                old = self._active
                self._active = allowed[0]
                log.info(f"[presets] speed lockout: switched from {old} to {self._active}")

    def toggle_focus_axle(self):
        with self._lock:
            self._focus_axle = "rear" if self._focus_axle == "front" else "front"
            log.info(f"[presets] editing axle -> {self._focus_axle}")

    def adjust(self, delta: float):
        with self._lock:
            axle = self._focus_axle
            val = self._presets[self._active][axle] + delta
            val = max(MIN_PSI, min(MAX_PSI, val))
            self._presets[self._active][axle] = val
            log.info(f"[presets] {self._active}.{axle} -> {val:.0f} psi")
        self._schedule_save()


# ============================================================
# Warning manager: per-corner punctures / over-pressure / stuck-deflate
# plus a system-wide possible compressor failure
# ============================================================
class WarningManager:
    """Tracks per-corner warnings (puncture, over-pressure, won't-deflate) and
    a system-wide possible compressor-failure flag. Each warning has an
    'active' state (the underlying condition is currently true) and a 'paused'
    state (the flashing has been silenced by a button press, but the condition
    hasn't necessarily resolved)."""

    # Warning kind -> the two body lines shown on the full-screen flash.
    PER_CORNER_KINDS = {
        "PUNCTURE": "PUNCTURE",
        "OVERPRESSURE": "OVER-PRESSURE",
        "DEFLATE_STUCK": "WON'T DEFLATE",
    }

    def __init__(self):
        self._lock = threading.Lock()
        # kind -> {corner_name -> {"active","paused"}}
        self._corner = {k: {} for k in self.PER_CORNER_KINDS}
        self._compressor_failure = {"active": False, "paused": False}

    @staticmethod
    def _set_flag(store, name, active, on_msg, off_msg):
        existing = store.get(name, {"active": False, "paused": False})
        if active and not existing["active"]:
            log.info(on_msg)
            store[name] = {"active": True, "paused": False}
        elif not active and existing["active"]:
            log.info(off_msg)
            store[name] = {"active": False, "paused": False}
        elif active:
            existing["active"] = True   # re-confirm, keep existing paused state
            store[name] = existing

    def _set(self, kind, name, active):
        label = self.PER_CORNER_KINDS[kind]
        with self._lock:
            self._set_flag(self._corner[kind], name, active,
                           f"[warning] {label} suspected: {name}",
                           f"[warning] {name} {label} warning cleared")

    def _has(self, kind, name) -> bool:
        with self._lock:
            entry = self._corner[kind].get(name)
            return bool(entry and entry["active"])

    # Convenience wrappers ------------------------------------------------
    def set_puncture(self, name, active):      self._set("PUNCTURE", name, active)
    def set_overpressure(self, name, active):  self._set("OVERPRESSURE", name, active)
    def set_deflate_stuck(self, name, active): self._set("DEFLATE_STUCK", name, active)
    def has_puncture(self, name) -> bool:      return self._has("PUNCTURE", name)

    def set_compressor_failure(self, active: bool):
        with self._lock:
            if active and not self._compressor_failure["active"]:
                log.info("[warning] POSSIBLE COMPRESSOR FAILURE suspected")
                self._compressor_failure = {"active": True, "paused": False}
            elif not active and self._compressor_failure["active"]:
                log.info("[warning] compressor failure warning cleared")
                self._compressor_failure = {"active": False, "paused": False}

    def pause_all(self) -> bool:
        with self._lock:
            paused_any = False
            for store in self._corner.values():
                for entry in store.values():
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
            items = []
            for kind, store in self._corner.items():
                for n, e in store.items():
                    if e["active"] and not e["paused"]:
                        items.append((kind, n))
            if self._compressor_failure["active"] and not self._compressor_failure["paused"]:
                items.append(("COMPRESSOR_FAILURE", None))
            return items

    def has_unpaused_active(self) -> bool:
        return bool(self.get_unpaused_active())

    def get_paused_highlights(self):
        """(set of corner names to show inverted, compressor_failure_paused: bool)"""
        with self._lock:
            names = set()
            for store in self._corner.values():
                names |= {n for n, e in store.items() if e["active"] and e["paused"]}
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


def corner_worker(corner: TyreCorner, presets: PresetManager,
                   warnings: WarningManager, tank: "TankController",
                   stop_event: threading.Event):
    """Runs continuously for one tyre. Speed-based lockout (see
    LockoutState) only ever gates the buttons/UI — it does NOT pause
    this loop. Pressure is checked and inflate/deflate keeps running
    at every speed, including above the Stage-2 threshold."""
    run_start = None       # when this corner started continuously trying to inflate
    deflate_start = None   # when it started continuously trying to deflate

    while not stop_event.is_set():
        try:
            corner.actual_psi = corner.sensor.read_psi()
        except Exception as exc:
            # No trustworthy pressure data (I2C error or out-of-range voltage):
            # close solenoids and wait. Self-healing — the next valid reading
            # clears the fault automatically (no permanent latch).
            if not corner.fault:
                log.warning(f"[{corner.name}] sensor fault: {exc}")
            corner.status = "FAULT"
            corner.fault = True
            corner.inflate.close()
            corner.deflate.close()
            run_start = deflate_start = None
            _interruptible_sleep(CHECK_INTERVAL_SECONDS, stop_event)
            continue

        if corner.fault:
            log.info(f"[{corner.name}] sensor recovered — resuming control")
            corner.fault = False

        # --- Absolute over-pressure guard (independent of preset target) ---
        if corner.actual_psi > HARD_MAX_PSI:
            if corner.status != "DEFLATE":
                log.warning(f"[{corner.name}] OVER-PRESSURE {corner.actual_psi:.0f}psi "
                            f"> {HARD_MAX_PSI:.0f} — force deflating")
            corner.status = "DEFLATE"
            run_start = deflate_start = None
            warnings.set_overpressure(corner.name, True)
            corner.inflate.close()
            corner.deflate.open()
            _interruptible_sleep(CHECK_INTERVAL_SECONDS, stop_event)
            corner.deflate.close()
            _interruptible_sleep(SETTLE_SECONDS, stop_event)
            continue
        warnings.set_overpressure(corner.name, False)

        target = presets.get_target(corner.axle)
        diff = target - corner.actual_psi

        if diff > TOLERANCE_PSI:
            corner.status = "INFLATE"
            run_start = run_start or time.monotonic()
            deflate_start = None
            warnings.set_deflate_stuck(corner.name, False)
            corner.deflate.close()
            corner.inflate.open()
            _interruptible_sleep(CHECK_INTERVAL_SECONDS, stop_event)
            corner.inflate.close()
            _interruptible_sleep(SETTLE_SECONDS, stop_event)

        elif diff < -TOLERANCE_PSI:
            corner.status = "DEFLATE"
            deflate_start = deflate_start or time.monotonic()
            run_start = None
            warnings.set_puncture(corner.name, False)
            corner.inflate.close()
            corner.deflate.open()
            _interruptible_sleep(CHECK_INTERVAL_SECONDS, stop_event)
            corner.deflate.close()
            _interruptible_sleep(SETTLE_SECONDS, stop_event)

        else:
            corner.status = "OK"
            run_start = deflate_start = None
            corner.inflate.close()
            corner.deflate.close()
            warnings.set_puncture(corner.name, False)
            warnings.set_deflate_stuck(corner.name, False)
            _interruptible_sleep(CHECK_INTERVAL_SECONDS, stop_event)

        # Struggling to inflate for too long? With a charged tank the supply is
        # known-good, so a corner that still can't reach target is a LOCAL
        # problem (puncture / stuck valve). If the tank itself is low or failed,
        # the TankController owns the system-wide compressor-failure warning, so
        # don't double-report here. KEEP TRYING either way (no permanent lock).
        if run_start and (time.monotonic() - run_start) > WARNING_TRIGGER_SECONDS:
            if tank.tank_ok():
                warnings.set_puncture(corner.name, True)
            run_start = time.monotonic()  # give it a fresh window before re-diagnosing

        # Struggling to DEFLATE for too long? That's a local problem (a jammed
        # deflate valve can't be a compressor issue) — flag it but keep trying.
        if deflate_start and (time.monotonic() - deflate_start) > WARNING_TRIGGER_SECONDS:
            warnings.set_deflate_stuck(corner.name, True)
            deflate_start = time.monotonic()


# ============================================================
# OLED display: 4WD outline + preset bar + warning flashing
# ============================================================
def load_splash_logo():
    """Loads the startup splash image, fitted to the full 128x64 screen
    and centered on a black canvas. Returns a PIL Image in mode "1", or
    None if no logo file is present / it failed to load — startup just
    skips the splash in that case."""
    if not os.path.exists(SPLASH_LOGO_PATH):
        log.info(f"[display] no splash logo at {SPLASH_LOGO_PATH} — skipping splash screen")
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
        log.warning(f"[display] could not load splash logo ({exc}) — skipping splash screen")
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

    if kind in WarningManager.PER_CORNER_KINDS:
        lines = ["**WARNING**", f"{CORNER_DISPLAY_NAME[name]} TYRE",
                 WarningManager.PER_CORNER_KINDS[kind]]
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
# Startup checks
# ============================================================
def check_pin_conflicts():
    """Fail fast at startup if the wiring map double-books a GPIO pin (only
    the actually-instantiated pins: active corners + compressor + buttons)."""
    used = {}

    def claim(pin, label):
        if pin in used:
            raise ValueError(f"GPIO pin conflict: '{label}' and '{used[pin]}' both use GPIO{pin}")
        used[pin] = label

    claim(COMPRESSOR_PIN, "compressor")
    for name in ACTIVE_CORNERS:
        claim(SOLENOID_PINS[name]["inflate"], f"{name} inflate")
        claim(SOLENOID_PINS[name]["deflate"], f"{name} deflate")
    claim(BUTTON_UP_PIN, "button UP")
    claim(BUTTON_DOWN_PIN, "button DOWN")
    claim(BUTTON_SELECT_PIN, "button SELECT")


# ============================================================
# Main
# ============================================================
def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    stop_event = threading.Event()

    check_pin_conflicts()

    # One ADS1115 per distinct I2C address — the corner sensors share
    # ADS1115_ADDR; the tank sensor may share that board or live on a second one.
    adcs = {}

    def get_adc(addr):
        if addr not in adcs:
            adcs[addr] = ADS1115(address=addr)
        return adcs[addr]

    sensors = {name: PressureSensor(get_adc(ADS1115_ADDR), SENSOR_CHANNELS[name])
               for name in ACTIVE_CORNERS}

    # Compressor is tank-pressure controlled, independent of the solenoids.
    tank_sensor = PressureSensor(get_adc(TANK_SENSOR_ADDR), TANK_SENSOR_CHANNEL)
    compressor = TankController(COMPRESSOR_PIN, tank_sensor)
    log.info(f"[compressor] tank-pressure control "
             f"(on <= {TANK_PRESSURE_CUT_IN:.0f}, off >= {TANK_PRESSURE_CUT_OUT:.0f} psi)")

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
            log.info("[lockout] UP ignored — Stage 2 (speed lockout)")
            return
        presets.adjust(PSI_STEP)

    def on_down():
        if warnings.has_unpaused_active():
            warnings.pause_all()
            return
        if lockout.stage() >= 2:
            log.info("[lockout] DOWN ignored — Stage 2 (speed lockout)")
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
            log.info("[lockout] SELECT(hold) ignored — Stage 2 (speed lockout)")
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
            log.info("[lockout] SELECT ignored — Stage 2 (speed lockout)")
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
                          args=(corners[n], presets, warnings, compressor, stop_event),
                          daemon=True)
        for n in corners
    ]
    threads.append(threading.Thread(
        target=display_loop, args=(device, corners, presets, lockout, warnings, stop_event), daemon=True))
    threads.append(threading.Thread(target=gps_worker, args=(lockout, stop_event), daemon=True))
    threads.append(threading.Thread(
        target=tank_worker, args=(compressor, warnings, stop_event), daemon=True))

    def shutdown(signum=None, frame=None):
        log.info("[system] Shutting down — closing all solenoids, stopping compressor...")
        stop_event.set()
        time.sleep(0.2)
        for c in corners.values():
            c.cleanup()
        compressor.cleanup()
        for adc in adcs.values():
            adc.close()
        presets.flush()
        device.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    for t in threads:
        t.start()

    log.info("[system] Running. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        shutdown()


if __name__ == "__main__":
    main()
