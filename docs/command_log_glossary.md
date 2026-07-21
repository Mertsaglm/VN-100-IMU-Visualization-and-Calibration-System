# VN-100 Command and Log Glossary

> This document explains, entry by entry, the commands sent manually to the sensor, their hex
> equivalents, and the dashboard console log messages, **verifying each one individually**. Every
> command's checksum was recomputed (with Python's `xor()`), and every hex sequence was checked
> byte-by-byte.
>
> ## ⭐ Source: FW 3.1.0.0 ICD
>
> This document is written against the **VN-100 Interface Control Document (Firmware v3.1.0.0)**
> (`STM32 Nucleo boards and VN 100 documents/VN100_ICD_fw3.1.0.0.pdf`); the ICD's example responses
> match the hardware's actual responses byte-for-byte (e.g. `$VNRRG,04,3.1.0.0*77`). Register
> semantics differ from FW 2.1 (UM001 Rev 2.22) at the points below — the full diff table is in
> [`docs/protocol.md`](protocol.md) §5.3:
> | Topic | FW 3.1.0.0 (used in this document) |
> |---|---|
> | `$VNSGB` target | **Reg 43** (Filter Startup Gyro Bias, ICD §3.3.5) |
> | Reg 46 (HSI status) | **does not exist in this firmware** (absent from the ICD register index) |
> | Reg 44 factory default | **`0,1,5`** (Off/Disable/5 — onboard HSI DISABLED) |
> | `$VNTAR` (Tare) | **does not exist in this firmware** → `$VNERR,04` (ICD §1.3) |
> | $VNERR code display | **hex** (0A/0B/0C); 01-08 unchanged |
>
> **All checksum and hex verifications below** apply to both ICD generations — the framing and the
> XOR checksum are identical; only register **semantics** change.
>
> Sources: `STM32 Nucleo boards and VN 100 documents/VN100_ICD_fw3.1.0.0.pdf` (**primary**),
> [`docs/protocol.md`](protocol.md), `pyvn100/capabilities.py`, `Core/Src/host_link.c`.
> UM001 Rev 2.22 (FW 2.1) is cited as the source for the older-generation register semantics, but
> its PDF is not included in this repository.

---

## 1. Two different frames — do not confuse them

This project uses **two separate command languages** (see `docs/protocol.md` §7):

| | Sensor command (to the VN-100) | Host command (PC ↔ STM32 bridge) |
|---|---|---|
| Format | `$<body>*<CS>\r\n` | `VN <COMMAND> [arg]\n` |
| Checksum | Yes — 8-bit XOR of the characters between `$` and `*`, 2-digit uppercase hex | None — plain text |
| Example | `$VNRRG,44*73\r\n` | `VN RAW $VNRRG,44*73\n` |
| Produced by | `pyvn100/protocol.py` (DIRECT mode) or the VN-100 itself (response) | `pyvn100/hostlink.py` (BRIDGE mode) |

> ### ⚠ Two requirements for typing commands manually into a terminal
>
> 1. **Tera Term: `Setup ▸ Terminal ▸ Transmit = CR+LF`.** `host_link.c` only processes a line once
>    it sees `\n` and discards `\r` — with Transmit set to `CR` only, the command **never completes**
>    and no response arrives. (Raw hex worked because the `0A` line ending was inserted by hand;
>    typing a string didn't — the main cause of the "my command isn't going through" feeling.)
> 2. **Send `$VNASY,0*4F` first** (mute the async stream) — while streaming, long commands can
>    collide with the sensor's ISR and drop bytes, producing `$VNERR,03` (root cause:
>    `docs/protocol.md` §8.1). The ICD recommends this; re-enable afterward with `$VNASY,1*4E`.

In **BRIDGE mode** (the normal usage in this project, over the STM32 bridge), every `$VN...` command
bound for the sensor is wrapped as `VN RAW <command>\n`; the STM32 (`host_link.c`) strips the
`VN RAW` prefix and forwards the rest to the sensor unchanged, appending `\r\n`. So sending
`VN RAW $VNRRG,44*73` results in `$VNRRG,44*73\r\n` actually going out over the wire — both express
the same command, one as seen on the PC→STM32 link, the other on the STM32→sensor link.

---

## 2. Registers used

All descriptions have been confirmed against the **FW 3.1.0.0 ICD** (read directly from the PDF via
`pdftotext`). Older FW 2.1 (UM001) references are called out separately wherever they differ.

| Reg | Name | Access | Size | Fields | Note |
|:--:|----|:--:|:--:|---------|-----|
| 1 | Model | R | — | model string | ✅ Bring-up **identity** read (which device is this?) |
| 2 | Hardware Version | R | — | hardware revision | ✅ Bring-up identity read |
| 4 | Firmware Version | R | 20 | `string[20]` | ✅ Bring-up identity read → **which ICD applies?** In the field: `3.1.0.0` |
| 43 | **Filter Startup Gyro Bias** | R/W | 12 | `GyroBias[3]` (rad/s) | **`$VNSGB` writes HERE** (ICD §3.3.5). Was Reg 74 in FW 2.1 |
| 6 | ADOR (Async Data Output Type) | R/W | 1 | `14`=VNYMR, `0`=off | ASCII output TYPE |
| 7 | ADOF (Async Data Output Frequency) | R/W | 1 | Hz | ASCII output RATE |
| 23 | **Magnetometer Calibration** | R/W | 48 B | `C[3×3]` + `B[3]` (Gauss) | The user's hard/soft-iron calibration. ICD §3.4.1: `X=C·(M−B)`. **Always active**, independent of the ApplyCompensation setting. Identical in both ICD versions. |
| 35 | VPE Basic Control | R/W | — | Enable/HeadingMode/… | |
| 44 | **Real-Time HSI Control** | R/W | 3 B | Mode, ApplyCompensation, ConvergeRate | Onboard (real-time) HSI control. **Factory default: `0,1,5` = DISABLED.** ICD §3.5.1 |
| ~~46~~ | ~~Magnetometer Calibration Status~~ | — | — | — | ⛔ **Does not exist in FW 3.1.0.0** — absent from the ICD register index (HSI is Reg 44 + Reg 47). The hardware returns an all-zero array: an undocumented legacy stub. Only meaningful on older v2.x. |
| 47 | **Real-Time HSI Results** | R (read-only) | 48 B | `Gain[9]` + `Bias[3]` | The solution the onboard HSI **computes itself**. ICD §4.5.1: its input is the magnetometer reading **after Reg 23 has already been applied** — the stages are chained. **Convergence is now measured from HERE** (since Reg 46 doesn't exist). |
| 75/76/77 | Binary Output 1/2/3 | R/W | — | AsyncMode, RateDivisor, OutputGroup, OutputField | High-rate binary output configuration |
| 84 | **Gyro Calibration** | R/W | 48 B | `C[3×3]` + `B[3]` (rad/s) | Corrects gyro scale/alignment/bias errors introduced by mounting. **Unrelated to `$VNSGB`**, which writes to Reg 43. |

### ⚠ Reg 84 and `$VNSGB` are **NOT THE SAME THING** (target register is **43**)

- `$VNSGB` (**Set Gyro Bias**) copies the current Kalman gyro-bias estimate into the **Filter
  Startup Gyro Bias** register. **The ID of this register depends on the firmware version:**
  **FW 3.1.0.0 → Register 43** (ICD §3.3.5) · FW 2.1 → Register 74 (UM001 §7.1.3).
  The code picks the correct one via `capabilities.gyro_bias_reg` and **verifies** the gyro-bias
  write by reading that register back. `VN SAVE` (`$VNWNV`) is required afterward to make it
  persistent.
- **Register 84** (named **Gyro Calibration** in FW 3.1.0.0) is entirely separate: a C-matrix +
  B-bias entered **manually** by the user, used to correct scale/alignment/bias errors from
  mounting, independent of factory calibration. Reading/writing Reg 84 is meaningful even without
  ever running `$VNSGB` — the factory default is the identity matrix plus zero bias
  (`1,0,0,0,1,0,0,0,1,0,0,0`).

### Reg 44 enum values (ICD FW 3.1.0.0 §3.5.1, Tables 3.56/3.57 — confirmed)

> Field names changed in FW 3: `HSIMode`→**Mode**, `HSIOutput`→**ApplyCompensation**
> (`NO_ONBOARD`→**Disable**, `USE_ONBOARD`→**Enable**). **Numeric values are unchanged** → wire-compatible.

| Field | Value | Meaning |
|------|:--:|-------|
| HSIMode | `0` | OFF — real-time HSI disabled |
| HSIMode | `1` | RUN — runs using the existing solution |
| HSIMode | `2` | RESET — resets the existing solution |
| HSIOutput | `1` | NO_ONBOARD — onboard HSI is not applied to the output (only Reg 23 has effect) |
| HSIOutput | `3` | USE_ONBOARD — onboard HSI (Reg 47 solution) is applied to the output |
| ConvergeRate | `1`…`5` | 1 = slow/precise (~60–90 s), 5 = fast/less precise (~15–20 s) |
| **Factory default (FW 3.1.0.0)** | **`0,1,5`** | **Off, Disable, 5 → onboard HSI DISABLED** (ICD §3.5.1; measured on hardware) |
| Factory default (older FW 2.1) | `1,3,5` | Run, Enable, fast (UM001 §8.3) |

---

## 3. Commands sent to the sensor (`$VN...`)

| Command | Meaning | Source |
|-------|--------|--------|
| `$VNRRG,<reg>*CS` | Read register | UM001 §5.1.1 |
| `$VNWRG,<reg>,<v1>,...*CS` | Write register | UM001 §5.1.2 |
| `$VNWNV*CS` | Write current settings to non-volatile memory (Write Non-Volatile) — must be called while the sensor is stationary, otherwise the Kalman filter can diverge (UM001 §5.1.3) | UM001 §5.1.3 |
| `$VNRFS*CS` | Restore factory settings + reset | UM001 §5.1.4 |
| `$VNRST*CS` | Software reset — all registers reload from NVM (or factory defaults if NVM is empty) and the Kalman filter converges from scratch | UM001 §5.1.5 |
| `$VNASY,0*CS` | Temporarily stop asynchronous output (ASCII **and** binary) — does NOT change any register setting, it only keeps the stream from interrupting register reads/writes | UM001 §5.1.8 |
| `$VNASY,1*CS` | Resume asynchronous output | UM001 §5.1.8 |
| `$VNSGB*CS` | Set Gyro Bias — copies the current gyro-bias estimate into **Reg 43** (was Reg 74 in FW 2.1; see warning above) | ICD §3.3.5 |
| `$VNTAR*CS` | Tare — ⛔ **does not exist in FW 3.1.0.0** → the sensor returns `$VNERR,04` (Invalid Command) | ICD §1.3 command list: VNRRG/VNWRG/VNWNV/VNRFS/VNRST/VNFWU/VNKMD/VNKAD/VNASY/VNSGB/VNBOM — **no VNTAR**. Only works on older v2.x. |

---

## 4. Host commands (PC → STM32 bridge, `VN ...`)

These go to the **STM32**, not the sensor; the STM32 (`Core/Src/host_link.c`) parses them and applies
them to the sensor.

| Command | Effect | Response |
|-------|------|-------|
| `VN PING` | — | `VNPONG` (this is what causes the connection mode to be auto-detected as BRIDGE) |
| `VN MODE ASCII\|BINARY` | Configures the sensor (reg 6/75) **and** selects the relay format sent back to the PC | `VNMODE ASCII\|BINARY` / `VNERR mode(ASCII\|BINARY)` |
| `VN FREQ <hz>` | Reg 7 in ASCII mode, Reg 75 RateDivisor in binary mode. Valid range is **1..200** | `VNACK` / `VNERR freq-range(1..200)` |
| `VN TYPE <ador>` | Sets Reg 6; only `0` or `14` are accepted | `VNACK` / `VNERR type(0\|14)` |
| `VN BAUD <baud>` | **Disabled** — a one-sided baud change would break the STM32↔sensor link | `VNERR baud-disabled` |
| `VN TARE` | Tare — the bridge forwards it for transparency, but FW 3.1.0.0 has no `$VNTAR`, so the sensor returns `$VNERR,04` (see §3). The dashboard automatically turns the button back off | `VNACK` (= only "written to UART") / `$VNERR,04` |
| `VN SAVE` | Sends `$VNWNV` | `VNACK` / `VNERR` |
| `VN FACTORY` | Sends `$VNRFS` | `VNACK` / `VNERR` |
| `VN RAW <text>` | Forwards the text to the sensor as-is (+`\r\n`). **Exception:** `$VNWRG,5,…` (baud write) is rejected — it would break the link | `VNACK` / `VNERR` / `VNERR baud-disabled` |

**Important:** `VNACK` only means **"the STM32 wrote the command to UART"** — it is not the sensor's
accept/reject decision. The real result is read from the sensor's echo in the console (`$VNWRG,...`)
or from `$VNERR,...` (see the "VNACK ≠ sensor accepted it" rule in `docs/protocol.md` §8.2).

---

## 5. Command/HEX verification table

Every row's checksum was recomputed with `xor()` and every hex sequence was checked byte-by-byte.

### 5.1 HSI read/write commands

| Operation | String | HEX | Verified |
|-------|--------|-----|:--:|
| Read Reg 44 (HSI control state) | `$VNRRG,44*73` | `24 56 4E 52 52 47 2C 34 34 2A 37 33 0D 0A` | ✅ |
| Read Reg 46 (HSI progress) | `$VNRRG,46*71` | `24 56 4E 52 52 47 2C 34 36 2A 37 31 0D 0A` | ✅ |
| Read Reg 47 (computed calibration) | `$VNRRG,47*70` | `24 56 4E 52 52 47 2C 34 37 2A 37 30 0D 0A` | ✅ |
| Reset+run onboard HSI | `$VNWRG,44,2,3,5*6E` | `24 56 4E 57 52 47 2C 34 34 2C 32 2C 33 2C 35 2A 36 45 0D 0A` | ✅ HSIMode=2(RESET), HSIOutput=3(USE_ONBOARD), ConvergeRate=5 |
| Freeze onboard HSI | `$VNWRG,44,0,3,5*6C` | `24 56 4E 57 52 47 2C 34 34 2C 30 2C 33 2C 35 2A 36 43 0D 0A` | ✅ HSIMode=0(OFF) but HSIOutput stays 3 → the algorithm stops but **the last converged solution keeps being applied** ("freeze") |
| Stop the stream | `VN RAW $VNASY,0*4F` | `56 4E 20 52 41 57 20 24 56 4E 41 53 59 2C 30 2A 34 46 0A` | ✅ (host bridge wrapper; the actual command reaching the sensor is `$VNASY,0*4F\r\n`) |
| Initial check to the STM | `VN PING` | `56 4E 20 50 49 4E 47 0A` | ✅ |

### 5.2 "Confirm HSI is on" sequence

| Step | String | HEX |
|------|--------|-----|
| 1 | `VN MODE ASCII` | (host, `\n`-terminated) |
| 2 | `VN RAW $VNWRG,44,1,3,5*6D` | `56 4E 20 52 41 57 20 24 56 4E 57 52 47 2C 34 34 2C 31 2C 33 2C 35 2A 36 44 0A` |
| 3 | `VN RAW $VNRRG,44*73` | as above |

Step 2: HSIMode=**1** (RUN — start the real-time algorithm), HSIOutput=3 (apply), ConvergeRate=5
(fast). Checksum `*6D` verified. Step 3 reads the result back to confirm HSIMode actually returned to
1 (a write request is not acceptance — see the rule in §4).

### 5.3 STM bridge sequential checklist

| Step | Command | Expected response | HEX verified |
|------|-------|-----------------|:--:|
| 1 | `VN PING` | `VNPONG` | ✅ |
| 2 | `VN MODE ASCII` | `VNMODE ASCII` | ✅ |
| 3 | `VN FREQ 10` | `VNACK` | ✅ |
| 4 | `VN RAW $VNRRG,4*47` | firmware version line (`$VNRRG,4,...`) | ✅ |
| 5 | `VN RAW $VNRRG,47*70` | Reg 47 HSI solution (3×3 C + 3×1 B) — convergence is measured **from here** | ✅ |

### 5.4 Register read commands (BRIDGE vs DIRECT framing)

For the register reads below, hex sequences are given for both "Bridge"
(`VN RAW $VNRRG,<reg>*CS\n`) and "Direct" (`$VNRRG,<reg>*CS\r\n`) framing — the only difference is
the host wrapper and line ending (`\n` vs `\r\n`); the checksummed body that reaches the sensor is
identical:

| Reg | Checksum | Content |
|:--:|:--:|--------|
| 6 | `*45` | Read ADOR |
| 7 | `*44` | Read ADOF |
| 23 | `*72` | Read mag compensation |
| 35 | `*75` | Read VPE Basic Control |
| 44 | `*73` | Read HSI control state |
| 46 | `*71` | Read HSI state — ⛔ this register **does not exist** in this firmware's ICD. However the hardware returns **14 zeros** instead of `$VNERR,08` (undocumented legacy stub). Checksum is correct; do not use this for decision-making |
| 47 | `*70` | Read computed HSI solution |
| 84 | `*7F` | Read gyro compensation |

### 5.5 Batch bring-up read block

```
VN MODE ASCII
VN RAW $VNRRG,23*72
VN RAW $VNRRG,44*73
VN RAW $VNRRG,46*71
VN RAW $VNRRG,47*70
VN RAW $VNRRG,84*7F
```

What each register in this block returns:
- Reg 23 → 12 floats (a 9-element C matrix + 3-element B bias), the mag compensation.
- Reg 44 → 3 values: HSIMode, HSIOutput, ConvergeRate.
- Reg 46 → ⛔ **does not exist in FW 3.1.0.0** (see §2). Older v2.x returned 7000/8000-range values;
  the code still tolerates both.
- Reg 47 → 12 floats, the solution computed by the onboard algorithm (read-only).
- Reg 84 → gyro compensation; **unrelated to `$VNSGB`** (see the correction in §2) — always a
  meaningful read, not only right after `$VNSGB`.

---

## 6. Console log format — how to read an example

Log format from `dashboard/app.py`: `→` = command going from PC to STM32 (TX), `←` = response/echo
coming from STM32 (RX), `·` = informational note, `[ACK]`/`[ERR]` = green/red tagging of the sensor's
`$VNWRG`/`$VNERR` echo.

A typical calibration sequence reads as follows:

| Line | Description |
|-------|----------|
| `dashboard started` / `link mode: STM BRIDGE (auto — VNPONG received)` | `detect_link()` sends `VN PING` as soon as it connects; since `VNPONG` comes back, BRIDGE mode is selected automatically (§7.6 protocol.md). |
| `→ VN FACTORY` / `← $VNRFS*5F [ACK]` | Host `VN FACTORY` → STM32 sends `$VNRFS` to the sensor; the sensor echoes its own `$VNRFS*5F` command (restore factory settings + reset). |
| `running bring-up self-test…` block | `pyvn100/selfcheck.py`'s automated check sequence: reads Reg 4 (firmware) + Reg 46, and runs stream/float-printf/firmware checks. |
| `VN RAW $VNRRG,23*72` / `$VNRRG,44*73` (paired read) → `VN MODE ASCII` → `$VNWRG,23,...*76` / `$VNWRG,44,0,1,3*68` (paired write) | When the calibration dialog opens, it first reads the current Reg23/44, switches to ASCII mode, then writes identity+zero (`1,0,0,0,1,0,0,0,1,0,0,0`) to Reg23 and `0,1,3` (HSIMode=OFF, HSIOutput=NO_ONBOARD, ConvergeRate=3) to Reg44 — establishing a **"clean baseline" (raw mag, HSI off)**. The response `$VNRRG,23,1,0,0,0,1,0,0,0,1,0,0,0*73 [ACK]` confirms this write was accepted (identity echo). |
| `$VNWRG,75,0,4,01,0128*7A`, `$VNWRG,06,14*59`, `$VNWRG,07,50*58` echoes | Echoes of the settings applied to the sensor by `VN MODE ASCII`'s multi-command sequence, defined in DIRECT mode (protocol.md §7.6): disable Reg 75 (binary), set Reg 6 (ADOR)=14, Reg 7 (ADOF)=50 Hz. |
| `→ VN RAW $VNWRG,23,<12 floats>*7B` / `← $VNERR,03*72 [ERR]` | If the calibration matrix write is rejected (code **3 = Invalid Checksum**, ICD §1.5), the checksum arrived corrupted at the sensor — root cause and fix are in `docs/protocol.md` §8.1. |
| `→ VN SAVE` / `← $VNWNV*57 [ACK]` | `$VNWNV` is sent — **writes the current register state to persistent memory.** |
| `→ VN RAW $VNRST*4D` / `← $VNRST*4D [ACK]` | Software reset confirmed. |

**Practical rule:** to confirm a write was truly accepted, don't rely on `VNACK` — check that
register's **`$VNRRG` read-back after the write.** Critical writes already do this automatically via
`write_register_verified()` (`docs/protocol.md` §8.2).

---

## 7. Known critical rules for this project (keep these in mind when reading logs)

- **`VNACK` ≠ "the sensor accepted it".** It only means "the STM32 wrote the command to UART." The
  actual accept/reject outcome comes from the sensor's echo in the console (`$VNWRG,...`) or from
  `$VNERR,...`.
- **Reg 46 does not exist at all in this firmware** (absent from the ICD register index). The
  hardware still returns an all-zero array (an undocumented legacy stub) → **no decision should
  ever depend on it**. The calibration wizard measures convergence from Reg 47 stability and
  progress from PC-side elapsed time.
- **Don't call a write "applied" before it's verified.** A successful `transport.write()` only means
  "bytes left the PC" — same as `VNACK`. Important writes are verified by reading them back
  (`VN100.write_register_verified`, `docs/protocol.md` §8.2).
- **115200 baud ceiling:** roughly 90 Hz max in ASCII, 270 Hz in binary — requesting a higher rate
  just hits the bandwidth limit, it is not an error.
- **Silent float-printf failure:** the `__asm__(".global _printf_float")` line in `main.c` plus the
  `-u _printf_float` line in `.cproject` guard against this; the bring-up self-test verifies it by
  checking for a realistic acceleration magnitude (`|accel|≈9.8 m/s²`).
