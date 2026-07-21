"""
Spec-constant (UM001 Rev 2.22) tests.

Values come from the VN-100 user manual, not the code -- a SPEC ANCHOR
against register/enum drift (e.g. the earlier HSIOutput 0/1/2 -> 1/3 fix
was exactly this kind of bug). Must match docs/protocol.md §5 and
calibration.md §2; the C side mirrors these in vn100_registers.h
(checked via host_selftest / build).
"""
import pytest

from pyvn100.registers import (ADOF_VALID, Reg, HSIMode, HSIOutput, HeadingMode,
                               AsyncType, AsyncMode)
from pyvn100 import binary


def test_register_ids_match_UM001():
    assert Reg.SERIAL_BAUD_RATE == 5
    assert Reg.ASYNC_DATA_OUTPUT_TYPE == 6        # ADOR
    assert Reg.ASYNC_DATA_OUTPUT_FREQ == 7        # ADOF
    assert Reg.YPR == 8
    # ICD Register Index: 17=Compensated Magnetometer, 19=Compensated Gyro.
    # A wrong constant here (e.g. `Reg.MAG_MEAS == 19`) would make this
    # "spec anchor" validate the wrong mapping.
    assert Reg.MAG_COMPENSATED == 17
    assert Reg.GYRO_COMPENSATED == 19
    assert Reg.MAG_CALIBRATION == 23              # Magnetometer Compensation (C+B)
    assert Reg.REF_FRAME_ROTATION == 26
    assert Reg.VPE_BASIC_CONTROL == 35
    assert Reg.HSI_CONTROL == 44                  # Magnetometer Calibration Control
    assert Reg.HSI_STATUS == 46                   # Magnetometer Calibration Status (FW v2.x)
    assert Reg.HSI_CALCULATED == 47               # Calculated Magnetometer Calibration
    assert Reg.BINARY_OUTPUT_1 == 75              # Binary Output 1
    assert Reg.BINARY_OUTPUT_2 == 76
    assert Reg.BINARY_OUTPUT_3 == 77
    assert Reg.GYRO_COMPENSATION == 84


def test_hsi_enums_match_UM001():
    # UM001 Table 2 (HSIMode) and Table 3 (HSIOutput)
    assert (HSIMode.OFF, HSIMode.RUN, HSIMode.RESET) == (0, 1, 2)
    # HSIOutput has only TWO values: 1=NO_ONBOARD, 3=USE_ONBOARD (0/2 DO NOT EXIST)
    assert HSIOutput.NO_ONBOARD == 1
    assert HSIOutput.USE_ONBOARD == 3
    assert HeadingMode.ABSOLUTE == 0 and HeadingMode.INDOOR == 2


def test_ador_and_binary_output_constants():
    assert AsyncType.OFF == 0
    assert AsyncType.VNYMR == 14                  # UM001 Table 28: setting 14 = VNYMR
    assert (AsyncMode.OFF, AsyncMode.PORT1, AsyncMode.BOTH) == (0, 1, 3)


def test_binary_group_and_field_mask_match_UM001():
    # Common group (0x01) + fieldMask 0x0128 = bit3(YPR)|bit5(AngularRate)|bit8(Accel)
    assert binary._GROUPS == 0x01
    assert binary._FIELDS_G1 == 0x0128
    assert binary._FIELDS_G1 == (1 << 3) | (1 << 5) | (1 << 8)
    assert binary.SYNC == 0xFA
    assert binary.FRAME_LEN == 42


def test_ADOF_enum_matches_ICD_table_3_9():
    """ADOF (Reg 7) is a CLOSED enum -- not a free range (ICD §3.2.4 Table 3.9).
    C counterpart: Core/Src/host_link.c `adof_is_valid()` -- same list; if one
    diverges, pc/host_selftest.c's 'VN FREQ 30/25' anchors break."""
    assert ADOF_VALID == (0, 1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200)
    # consistent with docs/protocol.md:117 (0 = stream off, noted there too)
    for hz in (1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200):
        assert hz in ADOF_VALID
    for hz in (3, 30, 60, 75, 150, 201):
        assert hz not in ADOF_VALID, f"{hz} must NOT be in the ADOF enum"


def test_dashboard_ASCII_HZ_list_is_a_subset_of_ADOF_enum():
    """The dashboard must only offer the user rates the sensor will actually ACCEPT."""
    pytest.importorskip("PySide6")
    from dashboard.app import ASCII_HZ
    for hz in ASCII_HZ:
        assert hz in ADOF_VALID, f"dashboard ASCII_HZ has a value outside the enum: {hz}"
