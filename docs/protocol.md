# VN-100 Communication Protocol — Interface Control Document (ICD / Single Source of Truth)

> This document is the **single source of truth (ICD)** for all framing/format/register rules used
> to talk to the VN-100. The C core (`Core/Src/vn100_protocol.c`, `vn100_binary.c`,
> `vn100_registers.h`) and the Python library (`pyvn100/protocol.py`, `binary.py`, `registers.py`)
> **must match it exactly**. If behavior changes, update this document first, then sync both sides.
>
> ## ⭐ Primary source: FW v3.1.0.0
>
> The target hardware runs **firmware 3.1.0.0**, so the primary source is the **VN-100 Interface
> Control Document (Firmware v3.1.0.0)**. UM001 Rev 2.22 (FW v2.1.0.0) is kept only as a
> legacy-generation reference — the five differences between the two ICDs are tabulated in §5.3.
>
> **Verification:** the ICD's example responses match the hardware's actual output **byte for
> byte** (e.g. `$VNRRG,04,3.1.0.0*77`, `$VNRRG,44,0,1,5*6B`).
>
> **References** (in `STM32 Nucleo boards and VN 100 documents/`):
> - `VN100_ICD_fw3.1.0.0.pdf` — ⭐ **PRIMARY**, the VN-100 IMU/AHRS ICD for FW v3.1.0.0
>   (VectorNav). *VectorNav Proprietary & Confidential — its license terms may restrict
>   redistribution; check before making the repository public.*
> - `STM32_Nucleo144_KullaniciKilavuzu_UM1974.pdf` — Nucleo-144 board manual (pinout, ST-Link VCP,
>   USART pins).
>
> UM001 Rev 2.22 and AN012 (Hard/Soft Iron calibration app note) are the sources behind the FW 2.1
> facts in §5.3, but their PDFs are **not included** here — get them from VectorNav if you need to
> work with FW 2.1 hardware.
>
> **Verification markers:** ✅ = confirmed from the ICD · 📟 = **measured on hardware** · 🔌 = can
> only be confirmed with the physical sensor (not yet measured). Every claim carries the marker
> for the evidence behind it — "the manual says so" is never conflated with "we observed it on
> hardware."

---

## 1. Layers

| Layer | Responsibility | Platform-independent? |
|--------|------------|:--:|
| Transport (LOW) | Physical transfer of bytes (UART/serial) | ❌ (platform-specific) |
| **Protocol (MID)** | **Framing, checksum/CRC, ASCII/binary decoding, command construction** | ✅ **(this document)** |
| API (HIGH) | Operations like `get_data`, `configure`, `set_output_mode`, `calibrate` | ✅ |

---

## 2. ASCII protocol (`$VN...`)  ✅

### 2.1 General frame
```
$<body>*<CS><CR><LF>
```
- `$`: start of message
- `<body>`: comma-separated fields (first field is the message type, e.g. `VNYMR`)
- `*`: checksum separator · `<CS>`: 8-bit XOR checksum, **2-digit uppercase hex** · `<CR><LF>`: `\r\n`

### 2.2 Checksum
The **8-bit XOR** of every character between `$` (exclusive) and `*` (exclusive).
> Verifiable example: `xor_checksum("VNYMR") == 0x5E`.

### 2.3 `$VNYMR` — Yaw/Pitch/Roll, Mag, Accel, Gyro  ✅ (ADOR type 14)
Field order (12 floats after the message type) — confirmed against ICD FW v3.1.0.0 §4.3.4
(Reg 27); identical in both ICDs:
```
$VNYMR,yaw,pitch,roll,magX,magY,magZ,accelX,accelY,accelZ,gyroX,gyroY,gyroZ*CS
```
| Field | Unit | Range |
|------|-------|--------|
| yaw, pitch, roll | degrees | yaw/roll [-180,180], pitch [-90,90] |
| magX/Y/Z | Gauss | — |
| accelX/Y/Z | m/s² | — |
| gyroX/Y/Z | rad/s | — |

---

## 3. Binary protocol (high speed)  ✅

### 3.1 Frame
```
0xFA | groups | [16-bit field bitmask (LE) for each active group] | payload | CRC16 (BE)
```
- `0xFA`: sync byte · `groups`: bitmask of which groups are present
- a 16-bit field mask for each active group · `payload`: raw (little-endian) data for the selected fields
- `CRC16`: computed over every byte **after** `0xFA`, big-endian

### 3.2 CRC-16 (VectorNav CRC16-CCITT, init=0, poly 0x1021)
On the receive side, recomputing the CRC over all bytes **excluding the sync byte (`0xFA`) but
including the CRC itself** must yield `0x0000`.
`crc16_ccitt()` is implemented **identically** in C and Python (equivalent to CRC-16/XMODEM;
verification: `crc16_ccitt(b"123456789") == 0x31C3`).

### 3.3 Configured output — Group 1 (Common)  ✅
Fields are transmitted in ascending order per the VectorNav Common-group bit order:

| Field | Bit | Size |
|------|:--:|-------|
| YawPitchRoll | 3 | 3 × float32 (12B) |
| AngularRate  | 5 | 3 × float32 (12B) |
| Accel        | 8 | 3 × float32 (12B) |

- `groups` byte = `0x01` (Common only)
- Group1 field mask = `(1<<3)|(1<<5)|(1<<8)` = **`0x0128`** (LE: `28 01`)
- payload = 36 bytes (9 × float32, little-endian) · total frame = 1+1+2+36+2 = **42 bytes**

The magnetometer is not included in the high-rate stream (it changes slowly; read via
ASCII/register instead). The layout is implemented identically by both `pyvn100.binary` and
`vn100_binary.c` and round-trip tested (`tests/test_binary.py`, `vn100_binary_selftest`).

---

## 4. Output mode selection — ASCII / Binary / dual mode  ⭐ (critical correctness fix)

ASCII output and binary output are **two separate, independent register subsystems** on the
VN-100. They are enabled/disabled independently and can run simultaneously or be switched between
on the fly.

### 4.1 ASCII output — Register 6 (ADOR) + Register 7 (ADOF)
- **ADOR (reg 6)**: async ASCII message TYPE. `14 = VNYMR`, `0 = off`. Command: `$VNWRG,6,14`
  (both implementations send the register without zero-padding — `VNWRG,{reg}` — whether the
  sensor also accepts the zero-padded `06` form has not been separately verified).
- **ADOF (reg 7)**: async ASCII FREQUENCY (Hz). Valid values: 1, 2, 4, 5, 10, 20, 25, 40, 50, 100,
  200. Command: `$VNWRG,7,50`.

### 4.2 Binary output — Register 75/76/77 (Binary Output 1/2/3)  ✅📟 (ICD FW v3.1.0.0 §3.2.8)
> **CORRECTION:** binary output is **not enabled via ADOR.** It is configured through separate
> Binary Output registers. (The old assumption — "set ADOR to binary" — was wrong; the frame
> *layout* was correct, only the *enabling method* was not.)

Each register takes 4 parameters:
| Field | Type | Description |
|------|-----|----------|
| AsyncMode | uint16 | `0` = off · `1` = serial port 1 · `2` = serial port 2 · `3` = both ports |
| RateDivisor | uint16 | Output rate = **IMU Rate (800 Hz) / RateDivisor** → `4` for 200 Hz, `8` for 100 Hz |
| OutputGroup | uint8 | Active-groups mask. `0x01` = Common |
| OutputField | uint16 (per group) | For Common, `0x0128` = YPR + AngularRate + Accel |

- Enable binary (200 Hz, single UART): `$VNWRG,75,2,4,01,0128` (this project uses TTL Serial Port
  2 / pins 8/9 → AsyncMode=2)
- Disable binary: `$VNWRG,75,0,...` (AsyncMode=0)

### 4.3 Dual mode — demo: ASCII, runtime: binary  ⭐
Because ADOR (reg 6) and Binary Output (reg 75) are independent:
- **Demo mode:** `ADOR=14` (ASCII on) + `reg75 AsyncMode=0` (binary off) → readable even in a
  plain terminal.
- **Runtime mode:** `ADOR=0` (ASCII off) + `reg75 AsyncMode=2, RateDivisor=4` (200 Hz binary).
- Switching is a runtime register write; **no reflash required.** HIGH API:
  `vn100_set_output_mode(ASCII|BINARY)`; host command: `VN MODE ASCII|BINARY`; a single toggle in
  the dashboard.

**Required receiver behavior — dual-mode auto-detecting parser:** the RX side identifies every
frame by its header (`$` → ASCII line, `0xFA` → 42-byte binary frame) — **mandatory**, not just a
convenience: even in binary mode, **command responses ($VNRRG/$VNWRG/$VNERR) still arrive as
ASCII**, interleaved on the wire, and a parser locked to a single mode would swallow them. (A
design requirement for both `vn100_rx_feed` and `pyvn100`'s `_feed`.)

### 4.4 Firmware version compatibility  ✅📟
The version-dependent risk lives in Reg 46, not Reg 75 (binary output): Reg 75 works the same
across both firmware generations (ICD §3.2.8; `$VNWRG,75,0,4,01,0128` was echoed back on hardware
📟), but Reg 46 does not exist in FW v3.1.0.0 (§5.3 #1).

**Current bring-up check:** on connecting to the sensor, an **identity triple** is read — Reg 1
(model), Reg 2 (hardware), Reg 4 (firmware) — to determine which ICD applies
(`pyvn100/selfcheck.py`, `capabilities.py`). The version string is treated as a **hint, not
proof**: for an unrecognized version the code falls back to the v3 profile but says so
**explicitly** via `known=False` — it never silently assumes correctness.

Safety design principle (unchanged): **the ASCII path is guaranteed to work on every firmware
version**; binary is enabled separately via reg 75.

---

## 5. Commands and register map (host → VN-100)

All commands follow the ASCII frame (`$...*CS\r\n`).

| Command | Meaning |
|-------|--------|
| `$VNRRG,<reg>*CS` | Read Register |
| `$VNWRG,<reg>,<v1>,<v2>,...*CS` | Write Register |
| `$VNWNV*CS` | Write Non-Volatile (persist settings to flash) |
| `$VNRFS*CS` | Restore Factory Settings |
| `$VNTAR*CS` | Tare — ⛔ **not present in FW v3.1.0.0** (absent from the §1.3 command list → returns `$VNERR,04`). Legacy v2.x only. |
| `$VNRST*CS` | Software reset |
| `$VNASY,0*CS` / `$VNASY,1*CS` | Pause / resume asynchronous output (ICD §1.3.9). Does NOT change any register; used to shield configuration writes from the live telemetry stream (see §8) |
| `$VNSGB*CS` | SetGyroBias — captures gyro bias into **Reg 43** while the sensor is stationary (Reg 74 in v2.1; §7 host command: `VN RAW $VNSGB`) |

### 5.1 Register IDs (ICD FW v3.1.0.0 — verified byte-for-byte ✅)
| Reg | Content | Access | Note |
|:--:|--------|:--:|-----|
| 1 | Model | R | ✅ bring-up **identity** read (which device?) |
| 2 | Hardware Version | R | ✅ bring-up identity read |
| 4 | Firmware Version (string[20]) | R | ✅ bring-up identity read → **which ICD applies?** (§5.3) |
| 5 | Serial Baud Rate | R/W | 9600…**921600** |
| 6 | Async Data Output Type (ADOR) | R/W | ASCII message type; **default 14** (=YMR), 0=off. The 2nd parameter `SerialPort` is **optional** → the plain `$VNWRG,6,14` form still works |
| 7 | Async Data Output Frequency (ADOF) | R/W | ASCII rate in Hz |
| 8 | Yaw/Pitch/Roll | R | |
| 17 | **Compensated Magnetometer** (calibrated) | R | ICD §4.4.1 |
| 19 | **Compensated Gyro** (calibrated) | R | ICD §4.4.3. ⚠ This register is **NOT the magnetometer** — the docs and code long assumed 19 was "Magnetic Measurements"; the magnetometer is actually **Reg 17** |
| 23 | **Magnetometer Calibration** (Hard/Soft Iron; C 3×3 + B 3×1; `C·(m−B)`) | R/W | ✅ user calibration; 48 B, identity by default. **Identical in both ICDs** |
| 26 | **Reference Frame Rotation** | R/W | axis alignment (requires write + WNV + reset) |
| 27 | **Yaw-Pitch-Roll & Compensated IMU** | R | ✅ **source of `$VNYMR`** (ADOR 14 → YMR header). Field order yaw, pitch, roll, mag, accel, gyro — **identical in both ICDs** |
| 35 | VPE Basic Control (Enable/HeadingMode/…) | R/W | |
| 43 | **Filter Startup Gyro Bias** | R/W | ✅ **`$VNSGB` writes here** (ICD §3.3.5). Was Reg 74 in v2.1 → gyro-bias verification reads back from this register |
| 44 | **Real-Time HSI Control** (Mode/ApplyCompensation/ConvergeRate) | R/W | ✅ onboard HSI; **factory default `0,1,5` = OFF** (§5.2) |
| 46 | ~~Magnetometer Calibration Status~~ | — | ⛔ **absent from the FW v3.1.0.0 ICD** (not in the register index). Hardware returns an all-zero payload (undocumented stub, 📟) → **no decision depends on it**; convergence is instead measured via Reg 47 stability. `decode_hsi_status` is retained only for legacy v2.x |
| 47 | **Real-Time HSI Results** (C 3×3 + B 3×1, 48B) | R | ✅ onboard solution. Its input is the mag reading AFTER Reg 23 has been applied (ICD §4.5.1) → the two stages chain together |
| 75/76/77 | **Binary Output 1/2/3** (AsyncMode/RateDivisor/OutputGroups/OutputTypes) | R/W | ✅ binary output. AsyncMode is now BITS (was an ENUM in v2.1) — the numeric values happen to overlap |
| 84 | **Gyro Calibration** (C 3×3 + B 3×1) | R/W | ✅ mounting-induced scale/alignment/bias. **Unrelated to `$VNSGB`** (which writes to Reg 43) |
| 206 | Legacy Compatibility Settings | R/W | v3 only; IMU temperature source + Tare **pin**. Does not restore Reg 46 |

### 5.2 Enum values (reg 44 — ICD FW v3.1.0.0 §3.5.1 ✅)
- **Mode**: `0` Off · `1` Run · `2` Reset — *same in both ICDs*
- **ApplyCompensation**: `1` Disable (onboard HSI not applied → Reg 23 only) · `3` Enable (applied)
  - ⚠ In FW v2.1 this field was named **HSIOutput**, with enum names **NO_ONBOARD/USE_ONBOARD**.
    **The numeric values are unchanged (1/3)** → wire-compatible; `pyvn100/registers.py` keeps the
    old names as aliases.
- **ConvergeRate**: `1` (slow, ~60-90 s, more accurate) … `5` (fast, ~15-20 s) — *unchanged*
- **Factory default (reg 44) = `0,1,5`** (Off, Disable, 5) ✅📟 — ICD §3.5.1 DEFAULT column;
  measured on hardware after two separate `$VNRFS` resets.
  **→ On this firmware, onboard HSI ships OFF from the factory.** (It was `1,3,5` = ON in FW v2.1.)
- HeadingMode (reg 35): `0` Absolute · `1` Relative · `2` Indoor

> C and Python share these IDs/enums via `vn100_registers.h` ↔ `pyvn100/registers.py`.
> Version-dependent behavior is centralized in **one place**: `pyvn100/capabilities.py`. Don't
> embed "if firmware is X, do Y" logic in the code — call `capabilities_for(fw)` instead.
> For details and the calibration workflow, see `docs/calibration.md`.

---

### 5.3 ICD differences — FW v2.1 → v3.1.0.0  ⭐ (five facts that directly affect this project)

Each row's evidence is labeled separately: **ICD** = stated in the document · **📟** = measured on
hardware.

| # | Topic | FW v2.1 (UM001 Rev 2.22) | **FW v3.1.0.0 (hardware in use)** | Evidence | Impact on this project |
|:-:|------|--------------------------|-------------------------------------|:-----:|----------------|
| 1 | **Reg 46** (HSI Status / bins) | Present; 7-8 bins, field layout not given in an official table | **ABSENT** — not in the register index; HSI is Reg 44 + Reg 47 instead | ICD + 📟 | 🔴 The calibration wizard's convergence gate used to depend on this register → it would **never open**. Convergence is now based on **Reg 47 stability**, and progress on **PC-side coverage** (`tools/coverage.py`). Hardware returns an all-zero payload for `$VNRRG,46` (undocumented stub) → no decision depends on it. |
| 2 | **Reg 44 factory default** | `1,3,5` (Run, Enable) — HSI **ON** | **`0,1,5`** (Off, Disable) — HSI **OFF** | ICD §3.5.1 + 📟 | 🟡 `calibration.md` §0's premise — "ships on → moving target" — is **wrong** for this hardware. Actually **good news**: the offline fit runs on a clean baseline out of the box. |
| 3 | **`$VNTAR`** (Tare command) | Present | **ABSENT** — not in the §1.3 command list (tare exists only as a *pin function* on Reg 206) | ICD | 🔴 The dashboard's Tare button used to fail silently (sensor returns `$VNERR,04`) → the button is now disabled based on firmware capability. |
| 4 | **`$VNSGB` target** | Reg **74** (UM001 §7.1.3) | Reg **43** — Filter Startup Gyro Bias (ICD §3.3.5) | ICD | 🟡 Gyro-bias writes are now verified by reading back from the correct register. |
| 5 | **`$VNERR` code format** | Decimal (10/11/12) | **Hex** (0A/0B/0C); 01–08 unchanged | ICD §1.5 | 🟢 `03` = Invalid Checksum, `04` = Invalid Command, `08` = Invalid Register — same in both. |

**WHAT DIDN'T CHANGE** (the real test of the architecture — the portable core survived a firmware
*generation* change intact): ASCII framing + XOR checksum · CRC-16 · **`$VNYMR` = ADOR 14 → Reg
27**, field order and units identical · Reg 6 ADOR (default **14** → `$VNYMR` streams out of the
box; the new `SerialPort` parameter is **optional**) · **binary Common Group bits YPR=3,
AngularRate=5, Accel=8 → `0x0128`**, 12+12+12=36 B payload, 42 B frame · Reg 75 field layout
(AsyncMode is now BITS rather than an ENUM but the numeric values overlap → port 2 = `2`) · Reg
23 / Reg 47 (48 B, 12 floats, identity default) · Reg 84 / 35 / 26 / 5 · commands `VNRRG, VNWRG,
VNWNV, VNRFS, VNRST, VNFWU, VNKMD, VNKAD, VNASY, VNSGB, VNBOM`.

**Open question (to be resolved on hardware, 🔌):** is Reg 46 truly dead, or does it populate
while HSI is running? Experiment: `VN RAW $VNWRG,44,1,3,5*6D` → wait 30 s → `$VNRRG,47*70` (has it
moved from identity?) + `$VNRRG,46*71` (still zero?). **The code does not wait on this answer** —
it has no dependency on Reg 46; the result will only update this table.

---

## 6. Cross-implementation sync rule
Every behavior in this document uses the **same test vectors on both sides**
(`tests/test_protocol.py` ↔ the C-side shared vectors). This prevents "correct on the PC, wrong on
the board" situations from the outset.

### 6.1 `error_count` definition (shared by both sides)

**`error_count` = number of frames that could NOT be decoded.** A frame is counted when:

| Case | Counted? |
|---|---|
| ASCII checksum / binary CRC mismatch | ✅ |
| A half-frame **dropped** due to a lost `'\n'` merging two lines, resynced on an embedded `$` | ✅ **once** |
| Message exceeded `VN100_MSG_MAX` (overflow) | ✅ |
| A stray `$`/`0xFA` was skipped but a valid frame was decoded afterward | ❌ (nothing lost) |
| A host line (`VNERR…`) — this is control-plane, not telemetry | ❌ |

> **Why this is written down:** during an audit, C counted `0` and Python counted `1` for the same
> input. Since this counter is shown on screen as "CRC n ERR" and used to diagnose link quality,
> both sides must count **exactly the same thing**. The Python side was aligned to this definition
> (`tests/test_parser_robustness.py`).

---

## 7. Host command protocol (PC ↔ STM32)

The dashboard sends commands to the STM32 (USART3 / ST-Link VCP); the STM32 (`host_link.c`)
decodes them and applies them to the VN-100. To avoid colliding with the VN-100's own `$VN...`
messages, host commands start with **`VN `** (no dollar sign) and end with `\n`. Python side:
`pyvn100.hostlink`.

| Command (PC→STM32) | Effect | Response |
|------------------|------|-------|
| `VN PING` | — | `VNPONG` |
| `VN MODE ASCII\|BINARY` | Configures the sensor (reg 6/75) **and** selects the relay format sent back to the PC. Only **`ASCII`/`BINARY`** accepted (else error); the last `VN FREQ` is preserved; the preserved rate is **clamped to 50 Hz** in ASCII | `VNMODE ASCII\|BINARY` / `VNERR mode(ASCII\|BINARY)` |
| `VN FREQ <hz>` | ASCII frequency (Reg 7) or binary RateDivisor; preserved across `VN MODE`. **Two-stage validation:** (1) range **1..200**, rejected outside it; (2) in **ASCII** mode it is additionally clamped to 50 Hz and checked for **ADOF enum membership** (§5.2 Reg 7 list — ICD Table 3.9; e.g. `VN FREQ 30` is in range but not a valid enum value → rejected). **BINARY** mode has no enum restriction: that path uses Reg 75's RateDivisor rather than Reg 7 (output = 800/divisor), so values like 80 Hz are valid | `VNACK` / `VNERR freq-range(1..200)` / `VNERR freq-adof(ICD Table 3.9)` |
| `VN TYPE <ador>` | Sets ADOR (Reg 6); only **`0`** (stop streaming) or **`14`** (VNYMR) accepted | `VNACK` / `VNERR type(0\|14)` |
| `VN BAUD <baud>` | **Disabled:** a one-sided baud rate change breaks the sensor↔STM32 link. A `VN RAW $VNWRG,5,...` (or `$VNWRG,05,...`) escape attempt is also rejected | `VNERR baud-disabled` |
| `VN TARE` | Tare — ⛔ **`$VNTAR` does not exist in FW v3.1.0.0** (§5.3 #3). The bridge command is still forwarded transparently and the STM32 returns `VNACK`, but **the sensor rejects it with `$VNERR,04`**. The dashboard disables the button based on `capabilities.has_tare`. Only actually works on v2.x | `VNACK` (= "written to UART" only) / `VNERR fail` |
| `VN SAVE` | Persist settings (VNWNV) | `VNACK` / `VNERR` |
| `VN FACTORY` | Factory reset (VNRFS) | `VNACK` / `VNERR` |
| `VN RAW <text>` | Forward the raw command directly to the sensor (+CRLF) | `VNACK` / `VNERR` |

**Implementation notes:**
- The USART3 RX interrupt on the STM32 is enabled in code (`main.c` + `USART3_IRQHandler`); no
  CubeMX changes required.
- The ISR only buffers bytes; commands are processed in the main loop (`host_link_process`) → no
  blocking inside the ISR.
- **Visibility of host responses on the PC:** the sensor's own `$VN...` echoes are shown in the PC
  console; the STM32's `$`-less `VNERR...` responses (`VNERR fail/mode/baud-disabled/…`) are also
  surfaced as **errors** on the PC side (`pyvn100/vn100.py` `_scan`) — since these are not sensor
  echoes, there is no other channel for them. `VNMODE` (mode confirmation) and `$VNRRG` register
  reads are also surfaced as informational console output; only `VNACK`/`VNPONG` are **deliberately
  dropped** as noise (command feedback comes from the sensor's own `$VN...` echo instead).
- **⚠ Known open issue (not yet fixed, 🔌 confirmed on hardware):** the `VN RAW` baud-lock check
  uses `strncmp(arg,"$VNWRG,",7)`, so a space **before** the `$` (a double space after `RAW` →
  `VN RAW ␣␣$VNWRG,5`; `strtok(NULL,"")` leaves a leading space in `arg`) can **bypass the guard**
  and write the baud rate to the sensor → potentially breaking the sensor↔STM32 link. The
  simulator's `.strip()` means it does NOT reproduce this bug (simulator is stricter than firmware
  here — a blind spot); the dashboard never sends free-form text, so normal use can't trigger it —
  the risk exists only via manual terminal/RAW input. Fix: strip leading whitespace before the
  `strncmp` check in the RAW branch. Whether the sensor accepts a space-prefixed `$VNWRG` can be
  tested with the harmless `VN RAW ␣␣$VNRRG,1` (a read).

### 7.6 Connection mode — STM bridge vs. direct USB-TTL  ⭐ (dual topology)

> **NOTE — terminology:** "dual mode" elsewhere in this document (§4) refers to **ASCII↔binary
> framing**. **Connection mode** here is a separate concept: which **topology** the PC uses to
> reach the sensor.

The dashboard works over two physical topologies; only the command **framing** differs (the
receive/parse side — `pyvn100/vn100.py` `_handle_line` — is identical in both, since sensor
responses always start with `$`; the bridge's `$`-less `VNERR/VNPONG` is a harmless superset):

| Mode | Topology | Command framing | Producer |
|-----|----------|-----------------|---------|
| **BRIDGE** | PC → STM32 (VCP) → VN-100 | `VN RAW $VNRRG,46*71\n`, `VN FREQ`, `VN MODE`… | `pyvn100/hostlink.py` |
| **DIRECT** | PC → USB-TTL → VN-100 (no STM32) | `$VNRRG,46*71\r\n` (raw, checksum+CRLF) | `pyvn100/protocol.py` |

- **Abstraction:** `pyvn100/link.py` implements `CommandLink` (Strategy) with
  `BridgeLink`/`DirectLink`; each logical command returns a wire-ready `list[str]` (a pure
  builder — sending/logging stays with the caller). The active strategy is `VN100.link`; the
  dashboard/dialogs/selfcheck all call through `vn.link.*`.
- **Opaque verbs expand into multiple commands (DIRECT):** `VN MODE ASCII` → `[disable reg75,
  reg6=14, reg7=hz]`; `VN MODE BINARY` → `[reg6=0, reg75 divisor]`; `VN FREQ` → reg7 in ASCII or
  the reg75 divisor in binary. Command ORDER matches `VN100.set_output_mode` exactly, to avoid an
  inconsistent output state mid-application.
- **Auto-detection:** `detect_link()` sends `VN PING` on connect. `VNPONG` is produced **only by
  the bridge** (the sensor never sends it), so its presence means BRIDGE and its absence means
  DIRECT. This check runs synchronously before the reader thread starts. Manual override: CLI
  `--bridge/--direct`, or the UI toggle.
- **Limitation:** detection only picks the **command language** — if the physical topology
  doesn't match (e.g. BRIDGE selected with no STM32 present, or its RX disconnected), commands go
  silently unanswered. In DIRECT mode, register reads still depend on the sensor's own `$VNRRG`
  response (a link/sensor issue, not a framing one).
- **Port discovery:** `find_stlink_port()` only matches the ST-Link VID (0x0483), so a direct
  USB-TTL adapter (FTDI/CP210x) is **not** found by `--port auto`; pass `--port COMx` explicitly
  for DIRECT mode.

---

## 8. Telemetry relay (STM32 → PC) — ground-station bridge

The STM32 broadcasts every new measurement it decodes from the VN-100 (over USART6) back to the PC
over the same VCP link (USART3).
```
VN-100 ──USART6──► STM32 (core parser: ASCII+binary) ──relay──► USART3/VCP ──► PC dashboard
```
- **Relay format follows the selected mode:** set via `VN MODE` (`host_link_t.out_fmt`);
  `vn_relay()` (main.c) sends a 42 B frame (`vn100_binary_encode`) in BINARY, or `$VNYMR`
  (`vn100_encode_vnymr`) in ASCII — the wire bytes genuinely match the selected format, not just
  what's shown on screen. The PC side's dual-mode-aware parser decodes whichever arrives and
  reports it via **`VN100.last_fmt`** (dashboard: "Stream: ASCII/BINARY"). The ASCII encoders
  `vn100_encode_vnymr()` (C) and `Vn100Simulator.encode_ascii()` (Python) match byte-for-byte,
  verified by `pc/host_selftest.c`. (Holds whenever the value is representable in float32, since
  both sides produce/consume float32.)
- The relay runs from the **main loop**, not an ISR; it always sends the **latest** data and drops
  intermediate frames if the loop can't keep up with the sensor rate — correct behavior for a live
  view. Could move to DMA-TX for higher throughput.
- **Bandwidth ceiling (hardware-imposed):** the STM32→PC link (USART3 / ST-Link VCP) is fixed at
  **115200 baud** (8N1 ≈ 11.5 KB/s). ASCII `$VNYMR` (~126 B/frame) caps at **~90 Hz**; binary
  (42 B/frame) caps at **~270 Hz**. Above that the relay drops frames (view stays live, just can't
  reach the requested rate) — hence the dashboard's rate list caps at ASCII ≤50, binary ≤200 (see
  `dashboard/app.py` `ASCII_HZ`/`BINARY_HZ`). The sensor→STM32 link (USART6, also 115200 by
  default) hits the same ceiling, so both hops would need a higher baud to go faster. The
  simulator has effectively infinite bandwidth, so this ceiling only shows up on REAL hardware.
- **TX timeout (🔌 confirmed on hardware):** relay/response TX uses `HOST_TX_TIMEOUT_MS = 30` ms
  (`main.c:43`). The `HAL_UART_Transmit` timeout is a single budget for the ENTIRE transfer; a
  lower value (e.g. 10 ms ≈ 115 B ceiling) would truncate the 12-float `$VNRRG,23/47` response
  (~160 B) on EVERY call, and the longer `$VNYMR` (~126 B) occasionally. The VCP applies no
  back-pressure → blocking is bounded by wire time, so this is safe. Still to confirm on hardware:
  that `VN RAW $VNRRG,23` returns the FULL response (12 floats + checksum).
- `%f` output requires newlib-nano's "float with printf" support; the code forces this two ways:
  `main.c`'s `__asm__(".global _printf_float")` plus the `.cproject` linker flag
  `-u _printf_float` (see README).

### 8.1 ⚠ Silent byte loss → `$VNERR,03` (observed on hardware 📟, root cause analyzed)

**Symptom:** a long command (e.g. the 132-character `VN RAW $VNWRG,23,<12 floats>*7B`) can be
rejected with **`$VNERR,03` (Invalid Checksum)**, while a short command right after (e.g. the
25-character `$VNWRG,44,0,1,3*68`) is accepted fine. The PC's checksum is verified
**mathematically correct**, and the line stays within both `HOST_LINK_LINE_MAX` (192) and the
sensor's 256 B input buffer → no overflow. So the PC sends correct bytes but the sensor receives
something different, and longer commands fail more often (see the probability estimate below).

**Root cause (proven by static analysis, hardware measurement still pending 🔌):**
- USART3 (PC→STM32) has NVIC priority **5**; USART6 (sensor) + DMA2_Stream1 have priority **1**
  (`main.c:186` vs `stm32f7xx_hal_msp.c:175`, `main.c:410`) → the host RX ISR cannot **preempt**
  the sensor ISR.
- The sensor ISR runs the ENTIRE parse inside the ISR context — **12× `strtof()`** per `$VNYMR`
  (`vn100_protocol.c:73` → `vn100.c:202` `vn100_rx_feed`).
- The STM32F7's USART has **no RX FIFO** (a single `RDR` register) → the tolerance is exactly
  **one byte period = 86.8 µs** @115200.
- On an overrun (ORE), `stm32f7xx_it.c:251` clears the flag and continues — **the lost byte is
  never counted or reported**. A byte dropped mid-line shortens the line, but the trailing `*7B`
  stays intact → the sensor computes a different checksum → **exactly `$VNERR,03`**.

**The probability matches what's observed:** at 50 Hz ASCII, a `$VNYMR` arrives every 20 ms. A
132-character command occupies the wire for **11.5 ms** → collision probability ≈ **57%**. A
25-character command takes **2.2 ms** → ≈ **11%**. This matches the asymmetry seen in the logs
(long commands fail often, short ones mostly succeed).

**Mitigation, without touching firmware (implemented):** configuration writes are wrapped between
`$VNASY,0` and `$VNASY,1` (ICD §1.3.9) to pause telemetry and close the collision window. Writes
are also **read back and verified**, with a **retry** on mismatch
(`VN100.write_register_verified`), so a dropped byte never produces a false confirmation.

**Other silent-loss paths** (none of them reported):
`host_link.c:37` — FIFO full → byte dropped · `vn100.c:76` — response queue full → **the newest
response** is dropped (4 slots, **3 usable**; consecutive reads have been observed on hardware to
skip a response 📟) · `main.c:529/537/557` — relay TX return codes are discarded.

### 8.2 Verified-write contract ⭐

> **`VNACK` does NOT mean "the sensor accepted it."** It only means "the STM32 wrote the command
> to UART."
> **A successful `transport.write()` does NOT mean "the sensor accepted it."** It only means "the
> bytes left the PC."

Every write whose outcome matters (calibration, gyro bias, factory reset) goes through
**`write_register_verified`**: `$VNASY,0` → `$VNWRG` → `$VNRRG` → wait for a **fresh** response
(filtered by timestamp) → compare with numeric tolerance (the sensor stores float32:
`1.002110` → `1.00211`) → **retry** on mismatch → `$VNASY,1`. An unverified write is treated as a
**failure**, and the UI never reports it as "applied."

Sensor errors are queried non-destructively via `VN100.errors_since(ts)`: even if the console's
`drain_responses()` empties the response queue, verification can still see the `$VNERR`
separately. This matters because console refresh and calibration verification run independently,
microseconds apart, in the same process — a single shared queue could let a `$VNERR` never reach
the calibration flow at all. Hence `_error_log` is kept separate and non-destructive.

> **Scope note:** `_error_log` also includes the **bridge's own** `VNERR fail` lines, not just
> `$`-prefixed sensor errors, so a "couldn't write to the sensor" report from the STM32 is also
> visible via `errors_since()`. Without this, commands with no read-back path (`$VNWNV`/`$VNSGB`)
> could show "✓ Saved" for a flash write that never actually happened.

---

## 9. Register read responses (sensor → PC bridge)

Calibration (Reg 44/47) and identity/capability probing (Reg 1/2/4) both require the PC to be able
to READ registers from the sensor:
```
PC → 'VN RAW $VNRRG,47' → STM32 (host_link) → sensor gets '$VNRRG,47' (USART6)
sensor → '$VNRRG,47,...' (USART6) → captured by the STM32 core → forwarded verbatim to PC (USART3)
PC → pyvn100.protocol.parse_vnrrg → registers.decode_mag_cal → hsi_solution_converged
```
Reg 46 is read over the same path but ONLY by `selfcheck`, for **informational** purposes
(`decode_hsi_status`); convergence/progress decisions are based on **Reg 47 stability + PC-side
coverage** (`tools/coverage.py`) — no decision depends on Reg 46 (§5.3 #1).
- The STM32 core places any complete line from the sensor that is **not `$VNYMR`**
  ($VNRRG/$VNWRG/$VNERR) into a response **queue** (`vn100_take_response`, 4 slots — an SPSC
  sentinel ring, **3 usable** — so back-to-back Reg 23+44 responses don't overwrite each other);
  `main.c`'s main loop drains the queue completely and forwards it to the PC verbatim → register
  responses reach the PC without disturbing the data stream. Slot size is 256 B (so the 12-float
  Reg 47 response isn't truncated).
- On the PC side, the `VN100` reader caches `$VNRRG` responses in `get_register(reg)`.
- Register commands are wrapped in **`VN RAW`** by the STM32 bridge. `pyvn100.hostlink` exposes:
  `read_reg`, `write_reg`, `hsi_reset/run/off`, `hsi_status`.

**Self-test:** `pc/host_selftest.c` compiles the core on the PC and verifies the
`$VNYMR → telemetry` and `$VNRRG → response mailbox` routing. The sim side (`SimTransport` +
`HSIEmulator`) emulates the full Reg 44/46/47 command-response flow → end-to-end testing without
hardware (`tests/test_hsi.py`).
