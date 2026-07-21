"""
tools/verify.py -- tests for the quality gate itself.

verify.py is the project's single-command verification gate. Contract under
test: if a step is SKIPPED (e.g. arm-none-eabi-gcc or GUI deps missing), the
output must NOT claim "ALL VERIFICATIONS PASSED" -- otherwise CI would look
like it proved something about firmware/GUI when it proved nothing. The GUI
precondition checks both PySide6 and pyqtgraph; missing either counts as
SKIPPED, not FAILED.

These tests fake `_run` and exercise main()'s decision logic -- no real build.
"""
import shutil

import pytest

from tools import verify


@pytest.fixture
def fake_gate(monkeypatch):
    """Fake `_run`: step name -> return value. No real compiler/pytest is invoked."""
    called = []          # (step name, command list) -- which step compiled what

    def make(results: dict, arm: bool = True, cppcheck: bool = True, gui: bool = True,
             missing_module: str | None = None):
        def _fake_run(name, cmd, env=None):
            called.append((name, cmd))
            for key, value in results.items():
                if key in name:
                    return value
            return True

        monkeypatch.setattr(verify, "_run", _fake_run)

        real_which = shutil.which

        def _which(prog):
            if prog == "arm-none-eabi-gcc":
                return "/fake/arm-none-eabi-gcc" if arm else None
            if prog == "cppcheck":
                return "/fake/cppcheck" if cppcheck else None
            if prog == "gcc":
                return "/fake/gcc"
            return real_which(prog)

        monkeypatch.setattr(verify.shutil, "which", _which)
        monkeypatch.setattr(verify.os, "makedirs", lambda *a, **k: None)
        def _find_spec(m):
            if missing_module is not None:
                return None if m == missing_module else object()
            return object() if gui else None

        monkeypatch.setattr(verify.importlib.util, "find_spec", _find_spec)
        return called

    return make


def test_one_FAILED_step_gives_nonzero_exit_code(fake_gate, capsys):
    """The gate's primary job: report failure with a non-zero exit code."""
    fake_gate({"pytest": False})
    rc = verify.main()
    assert rc != 0
    assert "THERE ARE FAILURES" in capsys.readouterr().out


def test_SKIPPED_step_means_ALL_VERIFICATIONS_PASSED_is_not_claimed(fake_gate, capsys):
    """REGRESSION: with arm-none-eabi-gcc and GUI deps missing (CI's actual
    state), output must NOT say 'ALL VERIFICATIONS PASSED' -- those steps
    never ran."""
    fake_gate({}, arm=False, cppcheck=False, gui=False)
    rc = verify.main()
    out = capsys.readouterr().out
    assert rc == 0, "the steps that did run passed -> exit code must stay 0"
    assert "ALL VERIFICATIONS PASSED" not in out, "full green was claimed while steps were skipped"
    assert "SKIPPED" in out
    for expected in ("arm_syntax", "arm_platform", "gui_import", "cppcheck"):
        assert expected in out, f"a skipped step must be listed by name in the summary: {expected}"


def test_no_skipped_steps_prints_FULL_GREEN(fake_gate, capsys):
    """Counter-check: if everything is present and passes, claiming full green is LEGITIMATE."""
    fake_gate({}, arm=True, cppcheck=True, gui=True)
    rc = verify.main()
    out = capsys.readouterr().out
    assert rc == 0
    assert "ALL VERIFICATIONS PASSED" in out


def test_ARM_platform_gate_also_builds_hal_msp(fake_gate):
    """stm32f7xx_hal_msp.c (ALL of UART/DMA/NVIC init) must be INSIDE the gate."""
    called = fake_gate({})
    verify.main()
    arm_platform = [cmd for name, cmd in called if "STM32 platform" in name]
    assert arm_platform, "the ARM platform step never ran"
    assert any("stm32f7xx_hal_msp.c" in a for a in arm_platform[0]), \
        "msp.c is outside the gate -- UART/DMA/NVIC init is never checked"


def test_GUI_gate_also_treats_pyqtgraph_as_a_precondition(fake_gate, capsys):
    """If pyqtgraph is missing, the GUI step must be SKIPPED, not FAILED (verify.py must not return 1)."""
    fake_gate({}, missing_module="pyqtgraph")
    rc = verify.main()
    out = capsys.readouterr().out
    assert rc == 0, "a missing GUI dependency must not FAIL the gate"
    assert "pyqtgraph" in out and "skipped" in out
