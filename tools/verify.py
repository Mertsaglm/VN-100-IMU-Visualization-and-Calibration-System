"""
tools.verify — Full verification with a single command: `python tools/verify.py`

What it runs (locally, without CI/GitHub):
  1. Python unit tests (pytest)
  2. Build the C core with the HOST C compiler (gcc/clang; 'gcc' = Apple clang on macOS)
     (-Wall -Wextra -Werror) + RUN the self-test (protocol + dual-mode parser, bit-for-bit vs Python)
  3. Build the PC-CLI (vn100_cli) (portability proof)
  4. If arm-none-eabi-gcc is on PATH, check the core for the ARM target with -fsyntax-only (optional)
  5. Static analysis with cppcheck if available (optional)

Exit code 0 = everything passed. Quality gate: a build warning is an error (-Werror).
"""
from __future__ import annotations

import importlib.util
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CORE = ["Core/Src/vn100_protocol.c", "Core/Src/vn100_binary.c", "Core/Src/vn100.c"]
CFLAGS = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", "-I", "Core/Inc"]


def _run(name: str, cmd: list[str], env=None) -> bool:
    print(f"\n=== {name} ===")
    print("  $ " + " ".join(cmd))
    try:
        return subprocess.run(cmd, cwd=ROOT, env=env).returncode == 0
    except OSError as exc:
        # Windows "App Control" (WDAC/Smart App Control) can block a freshly built
        # unsigned exe (WinError 4551); mark FAILED instead of crashing so the
        # summary table still prints.
        print(f"  [!] could not run: {exc}")
        print("      (If blocked by Windows App Control: add an exception for pc\\build "
              "or run the self-test on an unrestricted machine.)")
        return False


def main() -> int:
    try:                              # keep console output stable across encodings
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

    results: dict[str, object] = {}

    results["pytest"] = _run("Python unit tests (pytest)", [sys.executable, "-m", "pytest", "-q"])

    gcc = shutil.which("gcc")
    if gcc:
        os.makedirs(os.path.join(ROOT, "pc", "build"), exist_ok=True)
        ext = ".exe" if os.name == "nt" else ""
        selftest = os.path.join("pc", "build", "host_selftest" + ext)
        cli = os.path.join("pc", "build", "vn100_cli" + ext)

        ok = _run("Build C self-test (-Werror)",
                  [gcc, *CFLAGS, "pc/host_selftest.c", *CORE, "Core/Src/host_link.c",
                   "-o", selftest, "-lm"])
        if ok:
            ok = _run("Run C self-test", [os.path.join(ROOT, selftest)])
        results["c_selftest"] = ok

        results["c_cli_build"] = _run("Build PC-CLI (portability)",
                                      [gcc, *CFLAGS, "pc/vn100_cli.c", "pc/vn100_port_pc.c",
                                       *CORE, "-o", cli, "-lm"])
    else:
        print("\n[i] host C compiler (gcc/clang) not found — C verification skipped.")
        results["c_selftest"] = None
        results["c_cli_build"] = None

    # ARM target syntax check (optional): firmware must also build for Cortex-M7.
    # Probed via PATH only (no hardcoded install path); skipped silently if absent.
    # NOTE: -fsyntax-only is a syntax/type check, not a full link/flash — that only
    # happens in STM32CubeIDE with real hardware. Passing != "runs on the board".
    armgcc = shutil.which("arm-none-eabi-gcc")
    if armgcc:
        # (a) Portable core + host_link (our code) — STRICT (-Werror).
        results["arm_syntax"] = _run(
            "ARM syntax: core + host_link (-Werror, cortex-m7)",
            [armgcc, "-fsyntax-only", "-mcpu=cortex-m7", "-std=c11",
             "-Wall", "-Wextra", "-Werror", "-I", "Core/Inc",
             *CORE, "Core/Src/host_link.c"])
        # (b) STM32 platform layer (main.c / ISR / port) — checked against HAL headers.
        # HAL is vendor code, so -Werror is skipped here (header warnings from a
        # different arm-gcc/CubeCLT version shouldn't cause false failures); -fsyntax-only
        # still catches type/syntax errors in our own code, keeping main.c/it.c/port
        # regressions visible at the gate.
        hal_inc = [
            "-I", "Core/Inc",
            "-I", "Drivers/STM32F7xx_HAL_Driver/Inc",
            "-I", "Drivers/STM32F7xx_HAL_Driver/Inc/Legacy",
            "-I", "Drivers/CMSIS/Device/ST/STM32F7xx/Include",
            "-I", "Drivers/CMSIS/Include",
        ]
        results["arm_platform"] = _run(
            "ARM syntax: STM32 platform (main/ISR/port/MSP, with HAL headers)",
            [armgcc, "-fsyntax-only", "-mcpu=cortex-m7", "-std=c11",
             "-DSTM32F722xx", "-DUSE_HAL_DRIVER", "-Wall", *hal_inc,
             "Core/Src/main.c", "Core/Src/stm32f7xx_it.c", "Core/Src/vn100_port_stm32.c",
             # UART/DMA/NVIC init lives here; without it in the gate, a regression
             # would only surface in CubeIDE.
             "Core/Src/stm32f7xx_hal_msp.c"])
    else:
        print("\n[i] arm-none-eabi-gcc not found — ARM syntax check skipped (e.g. CI ubuntu).")
        results["arm_syntax"] = None
        results["arm_platform"] = None

    # GUI module import check (optional): catches dashboard syntax/import regressions.
    # Runs only if PySide6 is present (local .venv); skipped in CI (GUI deps absent
    # there). Checks both PySide6 and pyqtgraph (dashboard imports both) — checking
    # PySide6 alone would mark this "FAILED" instead of "SKIPPED" when only pyqtgraph
    # is missing, making verify.py return 1 as if it were a code regression.
    _gui_deps = ("PySide6", "pyqtgraph")
    _gui_missing = [m for m in _gui_deps if importlib.util.find_spec(m) is None]
    if not _gui_missing:
        results["gui_import"] = _run(
            "GUI module import (offscreen; if PySide6+pyqtgraph present)",
            [sys.executable, "-c",
             "import dashboard.app, dashboard.calibration_dialog, dashboard.gyro_bias_dialog"],
            env=dict(os.environ, QT_QPA_PLATFORM="offscreen"))
    else:
        print(f"\n[i] {', '.join(_gui_missing)} not found — GUI import check skipped (expected in CI).")
        results["gui_import"] = None

    cppcheck = shutil.which("cppcheck")
    if cppcheck:
        results["cppcheck"] = _run("cppcheck (static analysis)",
                                   [cppcheck, "--enable=warning,style", "--error-exitcode=1",
                                    "--quiet", "-I", "Core/Inc", *CORE, "pc/host_selftest.c"])
    else:
        print("\n[i] cppcheck not found — static analysis skipped (optional; a MISRA plugin is recommended).")
        results["cppcheck"] = None

    print("\n==================== SUMMARY ====================")
    failed = 0
    for name, ok in results.items():
        status = "PASSED" if ok is True else ("SKIPPED" if ok is None else "FAILED")
        print(f"  {name:14s}: {status}")
        if ok is False:
            failed = 1
    # Every SKIPPED step weakens the green: "ALL VERIFICATIONS PASSED" prints only if
    # nothing was skipped. Otherwise CI (no arm-none-eabi-gcc, no GUI deps) would print
    # it while firmware/GUI were never checked — implying more than was proved.
    skipped = [name for name, ok in results.items() if ok is None]
    c_skipped = (results.get("c_selftest") is None) or (results.get("c_cli_build") is None)
    print("=============================================")
    if failed:
        print("RESULT: THERE ARE FAILURES")
    elif c_skipped:
        print("RESULT: pytest PASSED — BUT gcc NOT FOUND: the C core was NOT built, self-test "
              "did NOT run. gcc is required for FULL verification (green = Python side only).")
    elif skipped:
        print(f"RESULT: THE STEPS THAT RAN PASSED — BUT {len(skipped)} STEP(S) WERE SKIPPED: "
              f"{', '.join(skipped)}.")
        print("        This green proves NOTHING about what the skipped steps cover "
              "(e.g. firmware if arm_* was skipped, dashboard if gui_import was skipped).")
    else:
        print("RESULT: ALL VERIFICATIONS PASSED (no step was skipped).")
    return failed


if __name__ == "__main__":
    sys.exit(main())
