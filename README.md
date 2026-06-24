# MarksCTI — 4WD Tyre Pressure Monitor & Auto-Inflator

A Raspberry Pi 5 central tyre inflation (CTI) controller for a 4WD. It
continuously monitors pressure at each corner, automatically inflates/deflates
to a selected preset, shows a live 4WD dashboard on an OLED, and locks out the
controls above road speed.

## Features

- **Per-corner pressure monitoring** via a DFRobot Gravity ADS1115 4-channel ADC
  (0.5–4.5 V = 0–150 psi sensors).
- **Automatic inflate/deflate** to front/rear target pressures using one
  inflate + one deflate solenoid per corner.
- **Four presets** — `ROAD` / `DIRT` / `SAND` / `CUSTOM` — each storing a front
  and rear target psi, editable from three buttons and persisted to
  `tyre_presets.json` across reboots.
- **OLED dashboard** (SSD1309, 128×64): 4WD outline with live per-corner
  pressure, active preset, and full-screen flashing warnings.
- **GPS speed-based lockout** (u-blox NEO-6M) in two stages so settings can't be
  changed while driving — but pressure regulation never stops. Serial port
  auto-reconnects if the GPS drops out.
- **Warnings** with silenceable full-screen flashes: puncture vs. possible
  compressor-failure heuristic, absolute **over-pressure** force-deflate guard
  (`HARD_MAX_PSI`), and a **stuck-deflate** ("won't deflate") warning.
- **Tank-pressure compressor control** — the compressor is decoupled from the
  solenoids and kept charging an air tank via a dedicated tank pressure sensor
  with hysteresis (on ≤ `TANK_PRESSURE_CUT_IN`, off ≥ `TANK_PRESSURE_CUT_OUT`).
  A tank that won't fill, or a tank-sensor fault, raises a compressor-failure
  warning (and a sensor fault stops the pump as a fail-safe). The corner
  solenoids draw from the tank on demand.
- **Self-healing sensor fault detection** — a sensor voltage outside its valid
  0.5–4.5 V band (disconnected/shorted) is treated as a fault (`ERR`, solenoids
  closed) instead of a bogus 0 psi, and clears automatically when it recovers.
  ADC reads are median-sampled, I2C-retried, and serialised across corners.
- **Configurable relay polarity** (`SOLENOID_ACTIVE_HIGH` /
  `COMPRESSOR_ACTIVE_HIGH`) for active-high or active-low driver boards.

Full system/wiring/setup detail lives in [`OVERVIEW.txt`](OVERVIEW.txt).

## Lockout stages

| Stage | Speed        | Buttons                                   |
|-------|--------------|-------------------------------------------|
| 0     | < 20 km/h    | Full access — all presets, all buttons    |
| 1     | 20–80 km/h   | ROAD/DIRT/SAND only (CUSTOM skipped)      |
| 2     | > 80 km/h    | Locked out (except silencing a warning)   |

Lockout gates the **UI only** — sensor monitoring and the inflate/deflate loop
run at every speed. No valid GPS fix → fail-safe to Stage 2.

## Hardware / wiring

See the module docstring at the top of [`tyre_inflator.py`](tyre_inflator.py)
for the full pin map. Summary:

| Function            | GPIO |
|---------------------|------|
| Compressor (MOSFET) | 4    |
| FL inflate / deflate| 17 / 27 |
| FR inflate / deflate| 22 / 23 |
| RL inflate / deflate| 24 / 25 |
| RR inflate / deflate| 5 / 6 |
| Button UP/DOWN/SELECT | 16 / 26 / 13 |
| ADS1115 / OLED (I2C)| SDA 2, SCL 3 |
| GPS RX (Pi RXD)     | 15   |

Corner sensors use ADS1115 channels A0–A3 (addr `0x48`). The **tank pressure
sensor** uses `TANK_SENSOR_ADDR` / `TANK_SENSOR_CHANNEL` — share the `0x48` board
on a free channel for bench testing, or fit a **second ADS1115 at `0x49`** once
all four corners occupy A0–A3.

> **Currently wired for testing:** only the `FL` corner. Add `"FR"`, `"RL"`,
> `"RR"` to `ACTIVE_CORNERS` in the script as each corner is wired.
>
> **I2C sensor supply:** a 0.5–4.5 V sensor read directly wants the ADS1115 at
> 5 V, which exposes the Pi's 3.3 V I2C pins to 5 V. Prefer 3.3 V on the bus; if
> the sensors are ratiometric, run them at 3.3 V too (and adjust `SENSOR_V_MIN` /
> `PSI_PER_VOLT`), otherwise add a divider or an I2C level shifter.

## Setup

```bash
sudo apt update
sudo apt install -y python3-lgpio python3-smbus i2c-tools fonts-dejavu-core
# Enable I2C and the serial port (login shell off, hardware on) via raspi-config, then reboot.
pip install -r requirements.txt --break-system-packages

i2cdetect -y 1     # confirm ADS1115 (0x48) and SSD1309 (0x3C/0x3D)
cat /dev/serial0   # should show $GPxxx NMEA sentences once the GPS has a fix
```

## Run

```bash
python3 tyre_inflator.py
```

### As a service (starts at boot)

```bash
sudo cp tyre-inflator.service /etc/systemd/system/
# Edit User/WorkingDirectory/ExecStart paths in the unit if not using /home/pi/MarksCTI
sudo systemctl daemon-reload
sudo systemctl enable --now tyre-inflator.service
```

## Tests

The pure control/state logic (`LockoutState`, `PresetManager`, `WarningManager`)
is unit-tested with the hardware libraries stubbed, so the suite runs on any
machine — no Raspberry Pi required:

```bash
python3 -m unittest discover -s tests
```

## Optional assets

Drop these next to the script (they're gitignored):

- `PixelOperator.ttf` — pixel font for the preset list (falls back to a built-in font).
- `splash_logo.png` — full-screen boot splash (any size; auto-fitted).

## Safety

This drives a live air system on a moving vehicle. Solenoids and the compressor
default to **off/closed** at startup and are closed again on clean shutdown
(`SIGINT`/`SIGTERM`). A failed or disconnected sensor faults its corner rather
than inflating blindly, and an absolute over-pressure guard force-deflates a
runaway corner regardless of its target.

⚠️ **Confirm your relay/MOSFET board polarity first.** Many opto-isolated relay
boards are active-low; if `SOLENOID_ACTIVE_HIGH` / `COMPRESSOR_ACTIVE_HIGH` don't
match your hardware, the "off" state at boot could energise everything. Set them
correctly and bench-verify nothing energises at power-on **before** connecting
air. Bench-test each corner before trusting it on a wheel, and make sure your air
hardware fails closed if power is lost.

The software tank control is **not** the only safeguard on the air system: fit a
**mechanical pressure switch** (backstop above `TANK_PRESSURE_CUT_OUT`) and a
**tank safety relief valve**, plus a check valve, tank drain and water trap. See
[`OVERVIEW.txt`](OVERVIEW.txt) for the full air-plumbing checklist.

## License

Apache-2.0 — see [LICENSE](LICENSE).
