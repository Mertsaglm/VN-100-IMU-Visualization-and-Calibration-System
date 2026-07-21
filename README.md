# VN-100 IMU Visualization and Calibration System

End-to-end data acquisition, visualization, and calibration system for the STM32 Nucleo-F722ZE +
VectorNav VN-100 IMU. **Corporate internship project.**

Built on a portable, layered driver architecture: the same protocol/API layer lives on both the
embedded (C) and PC (Python) sides — only the lowest (transport) layer is platform-specific.

## Features

- **Fast, low-latency data path:** UART DMA circular buffer + IDLE-line detection + half/full-transfer
  draining (lossless under normal load; under overload the DMA may drop the oldest bytes). The relay is
  **latest-sample** — it discards intermediate packets and forwards only the freshest one. Supports
  ASCII (`$VNYMR`) and binary (CRC-16) protocols.
- **Dual mode — ASCII for demos, binary for production:** the output mode is chosen at runtime
  (ASCII = reg 6, binary = reg 75; ≥200 Hz) with no reflash required; the receiving parser auto-detects
  each frame from its header, even in a mixed stream.
- **Ground-station bridge:** the STM32 re-publishes every decoded VN-100 measurement to the PC over
  USB-VCP in the **selected format** (ASCII `$VNYMR` or binary) — live data starts flowing as soon as
  the dashboard connects, in either mode.
- **Portable C core (`libvn100`):** a HAL-free protocol/API layer plus a thin STM32 port, with an
  on-board self-test **and the ability to build and run the same core on a PC** — self-test
  (`pc/host_selftest.c`) **and an interactive CLI** (`pc/vn100_cli.c` + `vn100_port_pc.c` + `Makefile`):
  `python vn100_simulator.py | pc/build/vn100_cli`.
- **Real-time dashboard (pyqtgraph):** **3D orientation view** (OpenGL sensor model, auto-falls back to
  QPainter if PyOpenGL/GL is unavailable — `dashboard/gl_view.py`), Euler/gyro/accel/mag plots, live
  readouts, rate/status, an ASCII/binary **mode toggle**, CSV **logging + playback**, and a one-click
  **bring-up check** (`pyvn100/selfcheck.py`).
- **Easy connection:** auto-detects the ST-Link port (`--port auto`), or lists available ports
  (`--list-ports`).
- **Two connection modes (auto-detected):** **STM32 bridge** (`VN RAW $VN…`) vs. **direct USB-TTL**
  (raw `$VN…`); selected automatically on connect via `VN PING` → `VNPONG`, or forced manually with
  `--bridge`/`--direct` (`pyvn100/link.py`). If the STM32 bridge misbehaves, the sensor can be wired
  directly and the same dashboard still works end to end, including calibration.
- **Bidirectional control + register access:** dashboard → (STM32 bridge / direct) → VN-100 for
  rate/type/save/factory-reset/raw-mode commands (tare is FW 2.x only — FW 3.1.0.0 has no `$VNTAR`, so
  the button disables itself), **plus register-read responses** (Reg 44/47, etc.) sent back to the PC.
  Every command sent and every response received (register reads, echoes) is shown in the
  **sensor console**. (`VN BAUD` is intentionally disabled — FW-U4.)
- **Correct calibration (two methods):** ① the **sensor's own HSI** — Reg 44 RESET → once converged,
  OFF → save; convergence is judged from the **stability of the Reg 47 solution**, with progress tracked
  PC-side since the fielded FW 3.1.0.0 lacks Reg 46 (see `docs/protocol.md` §5.3). ② Offline ellipsoid
  fit — disable HSI, collect raw data, fit, write Reg 23 — *the most reliable option, since it doesn't
  depend on the sensor's internal state at all*. A circular **sphere-coverage** visualization plus a
  maneuver checklist (✓/○) guides the process; results are saved permanently with `$VNWNV`.
- **Gyro bias (drift at rest):** measures bias/noise with the sensor stationary — reports a **live
  running σ** and **noise density (ARW)** for verification, **plus a SetGyroBias write** (`$VNSGB`) to
  the sensor. The SGB estimate is copied into the internal **Filter Startup Gyro Bias** register
  (persisted with `$VNWNV`) — **the register ID is version-dependent: Reg 43 on FW 3.1.0.0, Reg 74 on
  FW 2.1** (resolved by `pyvn100/capabilities.py`). User gyro compensation (scale/alignment/bias) is a
  separate register: **Reg 84**.
- **Hardware-free development:** a physics-based fake VN-100 (with **on-board HSI emulation**,
  hard/soft-iron, output-mode switching, SetGyroBias, and command checksum validation) plus a
  `SimTransport`. The simulator emulates **FW 3.1.0.0** capabilities (the older ICD can also be driven
  via `fw_version="2.1.0.0"`); backed by **255 unit tests**.

## Source layout

```
Core/                 STM32 firmware (libvn100 core + STM32 port + host_link)
pc/                    PC build of the core: host_selftest.c, vn100_cli.c, vn100_port_pc.c, Makefile
pyvn100/              PC library (protocol, binary, transport, vn100, simulator, hostlink, link, replay,
                      registers, types, selfcheck, capabilities ⭐ = the ONLY place FW-version differences live)
dashboard/            pyqtgraph GUI (app, calibration_dialog, gyro_bias_dialog, gl_view)
tools/                calibration (ellipsoid fit + pre-centering) + coverage + verify.py (single-command verification)
tests/                pytest (255)
docs/                 protocol.md (ICD), architecture.md, calibration.md, bringup_checklist.md
                      (live bring-up sequence), command_log_glossary.md (command/log reference),
                      working_commands_reference.md (command/HEX cheat sheet)
STM32 Nucleo boards   VN100_ICD_fw3.1.0.0.pdf ⭐ (the ACTUAL ICD for the fielded hardware — primary source;
and VN 100 documents/ VectorNav Proprietary & Confidential), STM32_Nucleo144_KullaniciKilavuzu_UM1974.pdf
vn100_dashboard.py    dashboard launcher
vn100_simulator.py    simulator CLI
```

## Documentation — which file, when?

| Document | What it's for / when to open it |
|-------|------------------------|
| [`docs/protocol.md`](docs/protocol.md) | **Single source of truth (ICD).** Frame format/checksum/CRC, register map, output modes, host command protocol. For any protocol question, **check here first** — the C and Python implementations follow it exactly. |
| [`docs/architecture.md`](docs/architecture.md) | Layered design, data flow, source tree. For "where does this piece belong?" questions. |
| [`docs/calibration.md`](docs/calibration.md) | Magnetometer (HSI) and gyro-bias calibration theory and workflows (onboard/offline, RAM↔flash, SetGyroBias). |
| [`docs/bringup_checklist.md`](docs/bringup_checklist.md) | Step-by-step live bring-up checklist (LED → port → bring-up check → dual-mode → calibration), with expected output at each step. |
| [`docs/command_log_glossary.md`](docs/command_log_glossary.md) | Reference for frequently used commands (hex, with checksum validation) and sample log lines. |
| [`docs/working_commands_reference.md`](docs/working_commands_reference.md) | HEX cheat sheet for commands sent manually (e.g. via Tera Term) — a condensed form of `command_log_glossary.md` for quick lookup. |

## Setup (PC side)

```bash
python -m venv .venv            # if 'python' is missing on macOS/Linux: python3 -m venv .venv
.venv\Scripts\activate          # Windows   (Linux/mac: source .venv/bin/activate)
pip install -r requirements.txt
```
> Note: Requires Python 3.11+. macOS/Linux usually only ships `python3` → run the first command as
> `python3 -m venv .venv` instead (once inside the venv, `python` becomes available again).

## Running

**Dashboard — simulation (no hardware):**
```bash
python vn100_dashboard.py --sim
```

**Dashboard — real hardware (STM32 Nucleo connected over USB):**
```bash
python vn100_dashboard.py --list-ports      # see available ports first
python vn100_dashboard.py --port auto        # auto-detect the ST-Link VCP
python vn100_dashboard.py --port COM5 --baud 115200
```

**Trying guided calibration without hardware** (the simulator sweeps the full sphere and emulates
the on-board HSI):
```bash
python vn100_dashboard.py --sim --sim-motion calibration
# Control -> "Magnetometer Calibration..." -> choose a method (recommended: the sensor's own HSI) -> Start
```

**Trying the gyro-bias tool** (simulator holds still with a known bias):
```bash
python vn100_dashboard.py --sim --sim-motion still
# Control -> "Gyro Bias (static)..." -> Start -> (optional) "Write to Sensor (SetGyroBias)"
```

**Replaying a recorded log:**
```bash
python vn100_dashboard.py --replay logs/vn100_YYYYmmdd_HHMMSS.csv
python vn100_dashboard.py --replay logs/altin.csv --replay-speed 4 --replay-loop
```

**Hybrid mode** (`--replay` + `--port` together) — measurements come from **the recording**, commands
go to **the real sensor**:
```bash
python vn100_dashboard.py --replay logs/altin.csv --port auto --replay-speed 8
```
Lets you replay a "golden" recording repeatedly to tune calibration without physically rotating the
sensor, then write the resulting solution to the real sensor. Only the **offline fit** method applies,
and the recording must have been captured in **RAW mode** (procedure + measured quality:
`docs/calibration.md` §4b).

**Selecting the connection mode manually** (default: auto-detected via `VN PING` → `VNPONG`):
```bash
python vn100_dashboard.py --port auto --bridge     # via the STM32 bridge
python vn100_dashboard.py --port COM5 --direct     # direct to the sensor (USB-TTL)
```

**Full verification — one command** (pytest + host-C `-Werror` self-test + PC-CLI; if
`arm-none-eabi-gcc` is available it also checks the firmware with ARM `-fsyntax-only`
[core + host_link + main/ISR/port], and if `PySide6` is available it imports the GUI modules):
```bash
python tools/verify.py
# or just the C side:  cd pc && make selftest      (interactive CLI:  make && python ../vn100_simulator.py | build/vn100_cli)
```

## Firmware (STM32)

Open the project in STM32CubeIDE → **Build** → **Run/Flash**. On startup:
- The protocol + binary **self-test** runs (red LED on failure).
- Blue LED (LD2): system ready. Green (LD1): valid packet. Red (LD3): error.
- The STM32 **relays** the measurements it decodes, in the **selected format** (ASCII `$VNYMR` or
  binary), from the VCP to the PC → the dashboard shows live data as soon as it connects with
  `--port` (the parser auto-detects either format).

> **Important — float printf (otherwise every `%f` value prints BLANK, a silent failure):** enforced
> **two independent ways** — `__asm__(".global _printf_float")` in `main.c` (in-code, equivalent to
> `-u`, survives code regeneration) **and** `-u _printf_float` under `.cproject` linker "Other flags".
> On the first flash, visually confirm a known float (e.g. `9.81`) does **not** print blank; also keep
> Project → Properties → C/C++ Build → Settings → MCU Settings → **"Use float with printf from
> newlib-nano"** checked.

## Status

| Area | Status |
|------|--------|
| PC library + dashboard (3D + playback + hybrid + mode toggle + STM bridge/direct connection) + calibration + gyro bias + dual-mode + tests (255) | ✅ Working, tested |
| **Protocol ICD (`docs/protocol.md`) aligned with the fielded hardware's ICD: FW 3.1.0.0** (`STM32 Nucleo boards and VN 100 documents/VN100_ICD_fw3.1.0.0.pdf`; binary reg 75, HSI 44/47, ApplyCompensation 1/3). UM001 Rev 2.22 (FW 2.1) is a secondary source for the older generation (PDF not included in this repo); the five differences between the two generations: `docs/protocol.md` §5.3 | ✅ |
| **Version-dependent behavior lives in one place** (`pyvn100/capabilities.py`): no Reg 46 · Reg 44 default `0,1,5` · no `$VNTAR` · `$VNSGB`→Reg 43 · $VNERR hex | ✅ the v2.1 profile is also preserved (`fw2`) |
| **C firmware core in sync with the ICD** (dual-mode parser, reg 75 binary, ApplyCompensation 1/3, SetGyroBias, `VN MODE`) | ✅ builds with gcc `-Werror`, self-test passes |
| **PC-CLI trio** (`vn100_port_pc.c` + `vn100_cli.c` + `Makefile`) — the same core runs on a PC | ✅ builds, decodes ASCII/binary streams |
| STM32 firmware (CubeIDE build + real sensor) | ⏳ code-complete and passes ARM `-fsyntax-only`; to confirm at bring-up: firmware version (Reg 4), reg 75 binary/Port 2 streaming, float-printf (`accel_z≈+9.81`) — see `docs/bringup_checklist.md` |
