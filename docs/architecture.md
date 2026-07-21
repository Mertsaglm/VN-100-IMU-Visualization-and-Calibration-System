# Architecture — VN-100 IMU System

## 1. Overview

```
   VN-100  ──UART (DMA circular + IDLE)──►  STM32F722 (Nucleo)
                                            │  portable core (libvn100)
                                            │  + host command channel
                                            ▼
                              USART3 / ST-Link VCP  (telemetry + commands)
                                            ▼
                          PC — pyvn100 library + pyqtgraph dashboard
```

Both platforms (STM32 and PC) share **the same layered design**. The goal: abstraction
makes it easy to port the driver to a different board ("develop on the PC, port to the board").

## 2. Layers (identical on both platforms)

| Layer | STM32 (C) | PC (Python) | Platform-independent? |
|-------|-----------|-------------|:--:|
| HIGH — API | `vn100.c` (`vn100_t`, `rx_feed`, commands) | `pyvn100/vn100.py` (`VN100`) | ✅ |
| MID — Protocol | `vn100_protocol.c`, `vn100_binary.c` | `pyvn100/protocol.py`, `binary.py` | ✅ |
| LOW — Transport/Port | `vn100_port_stm32.c` (HAL UART+DMA) | `pyvn100/transport.py` (Serial/Sim/Loopback) + `replay.py` (Replay/Hybrid) | ❌ platform-specific |

**Cross-cutting concerns (alongside the layers, PC side):**
- `pyvn100/capabilities.py` — **ICD generation differences live in ONE place.** `capabilities_for(fw)`
  resolves the `fw3`/`fw2` profile; version-dependent behavior is never hardcoded elsewhere — it's
  always looked up here. `probed()` lets an actual hardware probe override the documented profile
  (measurement beats assumption).
- `pyvn100/link.py` — **command-framing strategy.** `CommandLink` (ABC) → `BridgeLink`
  (`VN RAW $VN...`, via the STM32 bridge) / `DirectLink` (raw `$VN...*CS\r\n`, straight to the sensor).
  The active strategy lives at `VN100.link`; `detect_link()` auto-selects it via `VN PING`→`VNPONG`
  (only the bridge produces `VNPONG`). **Rule:** dashboard/dialog/selfcheck code always calls
  `vn.link.*`.

**Abstraction seam (C):** `vn100_port.h` — `{ write, millis, enter/exit_critical }`.
Porting to a new board means writing one new port file; MID/HIGH stay untouched.

**Push model:** incoming bytes are pushed into the core via `vn100_rx_feed()` / `VN100.poll()`
(from the IDLE ISR on STM32, from a reader thread on the PC). This keeps the core fully decoupled
from I/O.

## 3. Data flow (STM32) — ground-station bridge

```
USART6 RX → DMA circular buffer ──IDLE IRQ──► vn100_stm32_on_uart_idle()
   → vn100_rx_feed(core) → state machine → parse (ASCII/binary)
   → update vn100_data_t + on_packet callback (main.c: LED)
        │
   main.c main loop: vn_relay() ── out_fmt? vn100_encode_vnymr() / vn100_binary_encode() ──► USART3/VCP ──► PC dashboard
```
- **Dual mode (ASCII/binary):** the core parser auto-detects each frame from its header
  (`$` → ASCII, `0xFA` → binary); the sensor's output mode is chosen at runtime —
  **demo mode = ASCII** (reg 6), **normal operation = binary** (reg 75), no reflash required
  (docs/protocol.md §4).
- **Relay:** the STM32 re-transmits every new measurement to the PC in the currently selected
  format (`out_fmt`: ASCII `$VNYMR` or a binary frame); the receiving parser auto-detects either
  (docs/protocol.md §7). This closes the "dashboard ↔ STM32 ↔ VN-100" data path end to end.
- **Host commands:** travel the opposite direction over the same VCP (PC → STM32 → VN-100). The
  distinguishing factor is the prefix: host commands start with `VN ` (**no `$`**), while telemetry
  and sensor echoes start with `$VN...`. This distinction protects `$VNYMR` parsing — the backbone
  of the whole system. (STM32's own responses are likewise `$`-free: `VNACK`, `VNERR ...`, `VNPONG`.)
- LED control is the **application's** job (main.c callbacks) — the core has no GPIO code.

## 4. Source tree

```
Core/Inc, Core/Src/
  vn100_types.h            data structures / status codes         [PLATFORM-INDEPENDENT]
  vn100_registers.h        register IDs                           [PLATFORM-INDEPENDENT]
  vn100_protocol.h/.c      ASCII/checksum/CRC/commands             [PLATFORM-INDEPENDENT]
  vn100_binary.h/.c        binary codec                            [PLATFORM-INDEPENDENT]
  vn100_port.h             port interface (the seam)                [PLATFORM-INDEPENDENT]
  vn100.h/.c               high-level API                          [PLATFORM-INDEPENDENT]
  vn100_port_stm32.h/.c    STM32 HAL port (UART+DMA+IDLE)          [STM32-SPECIFIC]
  host_link.h/.c           PC<->STM32 command channel
  main.c                   application: init, callbacks, vn_relay, host_link_process
  stm32f7xx_it.c           USART6 IDLE + USART3 RX ISR bridges

pc/                        Portability proof: build+run the HAL-free core on the PC (Makefile)
  host_selftest.c          gcc self-test (protocol + DUAL-MODE parser + mixed stream; bit-for-bit match vs. Python)
  vn100_cli.c              console app driving the same core on the PC (stdin->rx_feed, ASCII+binary)
  vn100_port_pc.c          PC port (twin of vn100_port_stm32.c) — the ONLY platform-specific file  [PC-SPECIFIC]

pyvn100/                   PC library (same layers)
  types, registers, protocol, binary   (MID + data; binary_output/SetGyroBias commands)
  capabilities             (ICD generation differences in ONE place: capabilities_for(fw) -> fw3/fw2 profile; probed() lets a hardware probe override the documented profile)
  transport                (LOW: Serial / Sim / Loopback; list_ports/find_stlink_port)
  vn100                    (HIGH: VN100; DUAL-MODE auto-detecting parser, set_output_mode, set_gyro_bias)
  simulator                (fake VN-100: orientation physics + hard/soft-iron + HSIEmulator + output-mode/SetGyroBias emulation)
  hostlink                 (host command strings: MODE/FREQ/TYPE/HSI/gyro-bias — STM bridge framing)
  link                     (Strategy pattern: CommandLink + BridgeLink/DirectLink + detect_link — STM bridge vs. direct USB-TTL; see protocol.md §7.6)
  replay                   (ReplayTransport: pure playback, writable=False · HybridTransport: measurements from a recording + commands to the REAL sensor, writable=True/data_is_recorded=True)
  selfcheck                (bring-up self-check: streaming / float-printf / sensor identity (Reg 1/2) / FW-ICD capability profile; Reg 46 is INFORMATIONAL ONLY — no decision depends on it; `since` freshness gate; mode-agnostic via vn.link)

dashboard/                 pyqtgraph GUI (app: 3D orientation + plots + mode toggle + "Bring-up Self-Check"; gl_view: OpenGL 3D model/calibration point cloud + QPainter fallback; calibration_dialog; gyro_bias_dialog)
tools/                     calibration (ellipsoid fit) + coverage + verify.py (single-command verification)
tests/                     255 pytest cases (protocol, API, simulator, binary, hostlink, link/connection-mode, verified-write, calibration dialog, console visibility, 3D axes, calibration, coverage, HSI (FW3+FW2), dual-mode, playback, hybrid, spec anchoring, wire format, parser robustness, selfcheck, serial port reconnect, verify.py quality gate, dashboard buffer integrity, robustness/regression — including host-VNERR + 0xFA resync + sim parity + BRIDGE/DIRECT)
docs/                      protocol.md (ICD), architecture.md, calibration.md,
                           command_log_glossary.md (command + hex + log glossary),
                           working_commands_reference.md (command/hex cheat sheet), bringup_checklist.md (ordered bring-up checklist)
STM32 Nucleo boards and    VN100_ICD_fw3.1.0.0.pdf (PRIMARY source — the ICD for the hardware in the field,
VN 100 documents/          VectorNav Proprietary & Confidential) + STM32_Nucleo144_KullaniciKilavuzu_UM1974.pdf
pytest.ini                 scopes pytest collection to tests/ only
```

## 5. Correctness assurance

- **Single source of truth:** `docs/protocol.md`. Both the C and Python implementations follow it
  exactly.
- **Shared test vectors:** `xor("VNYMR")=0x5E`, `crc16("123456789")=0x31C3` — checked by both
  `tests/` (pytest) and the STM32 startup self-test (`vn100_protocol_selftest`,
  `vn100_binary_selftest`). This rules out "correct on the PC, wrong on the board" by construction.
- **Single-command verification (`python tools/verify.py`):** runs pytest + builds and runs the
  host-C core with gcc **`-Werror`** (self-test) + builds the PC CLI; if `arm-none-eabi-gcc` is
  available it also runs an ARM Cortex-M7 `-fsyntax-only` check on the firmware (core + host_link +
  HAL-based `main`/ISR/port), and imports the GUI modules if `PySide6` is available. A compiler
  warning counts as a failure (quality gate). Full firmware link/flash still happens in CubeIDE.
- **Build and run the core on the PC:** the HAL-free layers (protocol+vn100+binary) compile with
  gcc; `host_selftest.c` verifies the `$VNYMR` round trip, byte-for-byte equivalence with Python's
  `ascii_frame()`, and the **dual-mode parser** (binary frame + mixed stream); `vn100_cli` drives
  the same core live (`python vn100_simulator.py | pc/build/vn100_cli`) — concrete proof of the
  "same core on PC and STM32" principle.
- **Hardware-free testing:** the simulator (orientation physics + hard/soft-iron) plus
  `SimTransport` exercises the entire PC pipeline (including the dashboard and guided calibration);
  optional HIL is available for the STM32 side (feeding PC→USART6 over USB-UART).
