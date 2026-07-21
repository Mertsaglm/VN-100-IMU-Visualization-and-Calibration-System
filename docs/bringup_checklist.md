# Bring-Up Checklist — VN-100 Live Sensor

> **Purpose:** When connecting the sensor for the FIRST time, verify the data path **in order**
> to catch silent failures early before they waste time. Every step lists the **expected output**
> and **what to do if it fails**. Don't skip the order — resolve the first red flag before moving on.
>
> This checklist encodes known silent-failure signatures. Most steps are automated by the
> **"Bring-up Check"** button on the dashboard (`pyvn100.selfcheck`). Protocol reference:
> [`protocol.md`](protocol.md); command/log cheat sheet: [`command_log_glossary.md`](command_log_glossary.md).

---

## 0. Before connecting (single-COM-port rule)
- Make sure **every program that can hold the port is CLOSED** (Tera Term, VectorNav Control
  Center, etc.) — two programs can't open the same port. This is the first thing to check for
  "no data" issues.
- Wiring: **PG9=RX, PG14=TX**, shared GND. Flash the board (STM32CubeIDE) and power it up.

## 1. Power-on + LEDs (is the firmware alive) — VISUAL CHECK
| Expected | Meaning |
|---|---|
| **LD2 solid blue** | init + self-test passed, system ready (`main.c:202`) |
| **LD1 green toggling** | valid packets are streaming from the sensor (DMA/parse working) |
| LD3 red SOLID | self-test/init failure → `Error_Handler` (attach a debugger) |
| LD1 NOT toggling at all | **no data** → see Step 2 (cable/baud/DMA) |

> **DMA/DTCM note:** The DMA buffer lives in DTCM (linker RAM=0x20000000). This shouldn't be an
> issue on the F7; if it is, the symptom is exactly **LD1 never toggling** (not a silent failure).

## 2. Connect the port → dashboard
```
python vn100_dashboard.py --list-ports        # find the correct COM port
python vn100_dashboard.py --port auto          # auto-detect the ST-Link VCP
```
- Expected: **STREAMING** at the top, plots animating.
- If **NO DATA / LINK LOST**: wrong COM port, port already in use (Step 0), or wrong baud (Step 3).

## 3. One button: **Bring-up Check** (dashboard, COMMANDS card)
Runs automated checks; reports within **~1.5 s** plus console log lines:

| Item | ✓ (pass) | ✗/⚠ if it fails |
|---|---|---|
| **Telemetry stream** | "N packets, live, format=ascii" | ✗ → no stream: Steps 1-2, cable/baud |
| **float-printf / data** | "\|accel\|≈9.8 m/s² (~1g, real float)" | ✗ "\|accel\|≈0" → **float-printf not linked**: check `main.c:82` `__asm__(".global _printf_float")` + `.cproject -u _printf_float`; enable CubeIDE "Use float with printf", clean build |
| **Firmware version** | **"3.1.0.0"** — the baseline this project was validated against (ICD: `STM32 Nucleo boards and VN 100 documents/VN100_ICD_fw3.1.0.0.pdf`) | ⚠ "2.x"/"1.x" → **older ICD** applies: Reg 46 exists, HSI ships ON by default, `$VNTAR` exists, `$VNSGB`→Reg 74. Canonical diff table: `docs/protocol.md` §5.3; code-side handling: `pyvn100/capabilities.py` |
| **Reg 46 (HSI status)** | On v3, **expected: absent/zero array** — this register is undefined in the ICD, no logic depends on it | ⚠ On v2.x sensors, if unreadable: this register existed in the old ICD (informational only; doesn't affect calibration) |

> Note: with the unit stationary, visually confirm **once** that `accel_z ≈ +9.81` (positive, a real
> number) — this is the most reliable proof that float-printf is working.

## 4. Dual mode (ASCII ⇄ binary) — demo/working mode + **Port 2 verification (most critical)**
- **MODE** button: ASCII (demo, ~≤50 Hz) ↔ BINARY (working mode, ~≤200 Hz).
- Pressing **BINARY → data must KEEP STREAMING** and the **Mode/Stream** label at the top must
  switch to **BINARY** (this confirms the PC is actually decoding `0xFA` frames). ✅ If both hold:
  **the sensor is broadcasting binary on TTL Serial Port 2, and the STM32 is relaying it →
  AsyncMode=2/Port 2 CONFIRMED.**
  ✗ If data **FREEZES** on switching to BINARY: the sensor is broadcasting on the wrong port
  (AsyncMode), or Reg 75 isn't supported (very old firmware) → check wiring and firmware version.
  (The code already writes `AsyncMode=2` — see `vn100_registers.h`.)
  Note: Reg 75 is still present in the v3.1.0.0 ICD with the same field layout (§5.3 "UNCHANGED").
- The **Stream: ASCII/BINARY** label at the top reflects the format actually being decoded (settles
  after a few packets).
- "I selected 200 but I'm seeing 90" is NOT a bug: it's the **115200 baud ceiling** (ASCII ~90,
  binary ~270 Hz).
- **Mag = 0 in binary mode** (by design) → calibration requires ASCII (the wizard switches
  automatically). The dashboard's **MAGNETOMETER plot** also goes blank in binary mode and shows
  "BINARY · mag in ASCII" (it won't draw a flat zero line); switching to ASCII fills in the Earth
  field components. The **`|M|` readout** top-right = √(Mx²+My²+Mz²) field magnitude: individual
  components move as you rotate the sensor, but **`|M|` should stay ~0.45 G** — the most reliable
  confirmation that the magnetometer is live and healthy (large swings mean calibration is needed).

## 5. Magnetometer calibration (wizard)
Open **Mag Calibration…** first. Two paths; the wizard enforces the RAM↔flash distinction:
1. **Onboard (wizard default):** Start → slowly rotate the sensor **through every orientation**;
   once **coverage ≥ 55%** and **the sensor's solution (Reg 47) has left identity and settled across
   consecutive readings**, **Apply ▸ Preview** appears. Inspect it live, then **Save (persist)** or
   **Discard**.
   ⚠ **The bin indicators stay empty on this firmware** — expected (Reg 46 doesn't exist on FW
   3.1.0.0); don't wait on them. If it fails to converge, **switch to offline** (offline doesn't
   depend on the sensor's internal state).
2. **Offline:** collect sufficient coverage (figure-8 + all 6 faces) → **Fit**. If the fit is
   **unreliable** (planar/ill-conditioned), the tool reports "NOT WRITTEN to Reg 23" — rotate more
   evenly and retry.
- **Apply ▸ Preview = RAM** (temporary), **Save = flash (`$VNWNV`)**. The distinction is shown on
  screen.
- **Discard** restores the pre-session Reg 23/44 values; if no snapshot exists, it leaves the
  sensor untouched (and warns you).
- Every critical write is **read back from the sensor to verify** (`$VNASY,0` → write → read back →
  tolerance check → retry → `$VNASY,1`). If the readback doesn't match, the tool does NOT report
  "applied/saved" — it explicitly reports **"sensor did not verify"**. A VNACK ≠ the sensor accepted
  the value — the console echo is now only **supplementary** visibility, not a gate.

## 6. Gyro bias (static) — drift while stationary
Open **Gyro Bias…** → fix the sensor to a **rigid, vibration-free** surface:
1. Once it reports "Settling/Stable" and collects ~500 samples, **Write to Sensor** becomes active.
2. At write time, stability **and freshness** are re-verified; if the data is moving or stale, the
   write is **aborted** (fail-closed). On success: `$VNSGB` + `$VNWNV` (persisted).

## 7. When finished / logging
- Use **START LOGGING** to record to CSV (`logs/vn100_*.csv`); review later with `--replay`
  (no sensor needed).
- No persistent setting is written to flash without `$VNWNV` (Save / VN SAVE) → a power cycle is
  always a safe fallback (reverts to the last flashed values).

## 8. Hardware verification of firmware hardening
These steps validate the firmware's hardening measures against the real sensor.

> **⚠ How to send these:** The dashboard console is **read-only** — you cannot send free-form
> `VN RAW …` / `VN FREQ 300` commands (only the fixed buttons/menus are available). To send these
> commands, **close the dashboard** (single-port rule) and open the port with a small Python
> script instead:
> ```python
> from pyvn100 import VN100, SerialTransport
> import time
> vn = VN100(SerialTransport("/dev/tty.usbmodemXXXX", 115200))   # use YOUR port (--list-ports)
> vn.start_reader(); time.sleep(0.5); vn.drain_responses()
> vn.send_raw("VN RAW $VNRRG,23*72\n")          # ← command under test
> time.sleep(0.7); print(vn.drain_responses(), vn.get_register(23))
> vn.stop_reader()
> ```
> (The ASCII 50 Hz clamp test is done via the MODE button — no script needed.)

1. **TX timeout (30 ms):** send `VN RAW $VNRRG,23*72` → confirm the console shows the **FULL**
   response (12 floats + checksum). A truncated/partial response means the timeout is still too
   short.
2. **ASCII 50 Hz clamp:** while streaming binary at 200 Hz, press **ASCII** → confirm the stream
   drops to ~50 Hz and stays **clean** (no truncated frames).
3. **FREQ range:** `VN FREQ 300` → should return `VNERR freq-range` (not a silent accept).
4. **Baud-change lockout:** `VN RAW $VNWRG,5,921600` (SINGLE space) → should return
   `VNERR baud-disabled`, and the sensor must NOT be written (the guard is working → link stays
   safe).
5. **⚠ Baud-lock leading-space gap (KNOWN, not fixed):** `VN RAW ␣␣$VNRRG,1` (**DOUBLE** space after
   RAW; reg 1 = **harmless MODEL READ** — do not try `$VNWRG,5` here). A response
   (`$VNRRG,1,VN-100…`) means the sensor **tolerates a leading space**, so a double-spaced
   `$VNWRG,5` could bypass the guard and change the baud rate, dropping the link
   (`docs/protocol.md` §7) — if so, never send `$VNWRG,5` with odd leading whitespace (or patch
   the RAW guard to reject leading-space bypasses). No response means this isn't practically
   exploitable.
6. **Checksum-less RAW:** does the sensor accept a `VN RAW $VNRRG,1` **without** a checksum?
   (The simulator tolerates it; real-sensor behavior is unknown — if rejected, use the checksummed
   form: `$VNRRG,01*72`.)
   ⚠ Run this test **with Reg 1**: the response (`$VNRRG,01,VN-100…`) gives an unambiguous yes/no.
   Reg 46 is undefined in this ICD and would confound the result — you couldn't tell "checksum
   rejected" from "register quirk."

---

### Quick decision tree (first red flag)
- **No LED/no data at all** → check wiring (PG9/PG14/GND) + single-port rule + baud (verify in
  Control Center).
- **Data present, values read 0.000** → float-printf (Step 3).
- **Calibration shows "mag 0"** → you're in binary mode; switch to ASCII (wizard does this
  automatically).
- **Onboard calibration won't converge** → move away from metal/magnets; if still stuck, use
  **offline**.
- **"I saved it but it's gone after power cycle"** → it was only written to RAM; use
  **Save (persist) / VN SAVE**.

> **Golden rule:** VNACK only means "the command was written to UART." Confirm a setting actually
> **stuck** via the sensor's echo in the console (`$VNWRG…`). If unsure, power-cycle — flash is
> always safe.
