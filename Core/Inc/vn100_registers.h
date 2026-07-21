/**
  ******************************************************************************
  * @file    vn100_registers.h
  * @brief   VN-100 register IDs (platform-INDEPENDENT)
  ******************************************************************************
  * PRIMARY SOURCE: VectorNav ICD **FW 3.1.0.0** (STM/VN100_ICD_fw3.1.0.0.pdf) -
  * the ICD for the hardware actually in use. UM001 Rev 2.22 / FW v2.1.0.0 is
  * HISTORICAL reference only; on conflict, the v3 ICD WINS (see the "five
  * facts" table in docs/protocol.md, section 5.3).
  * Do not embed version-DEPENDENT behavior here — query pyvn100/capabilities.py
  * instead. Full mapping + verification: docs/protocol.md, section 5.
  * pyvn100/registers.py holds the same values.
  ******************************************************************************
  */
#ifndef VN100_REGISTERS_H
#define VN100_REGISTERS_H

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    VN100_REG_MODEL_NUMBER      = 1,
    VN100_REG_FIRMWARE_VERSION  = 4,
    VN100_REG_SERIAL_BAUD_RATE  = 5,   /* Reg 5 */
    VN100_REG_ASYNC_OUT_TYPE    = 6,   /* Reg 6 - ADOR (ASCII message type) */
    VN100_REG_ASYNC_OUT_FREQ    = 7,   /* Reg 7 - ADOF (ASCII Hz) */
    VN100_REG_YPR               = 8,   /* Reg 8 - Yaw/Pitch/Roll */
    /* NOTE: Reg 19 is the GYRO, NOT the magnetometer. The old name
       VN100_REG_MAG_MEAS=19 ("Magnetic Measurements") was wrong - the
       magnetometer is Reg 17. Source: STM/VN100_ICD_fw3.1.0.0.pdf
       "Register Index" (17/18/19/20). */
    VN100_REG_MAG_COMPENSATED   = 17,  /* Reg 17 - Compensated Magnetometer */
    VN100_REG_GYRO_COMPENSATED  = 19,  /* Reg 19 - Compensated Gyro */
    VN100_REG_MAG_CALIBRATION   = 23,  /* Reg 23 - Magnetometer Compensation (Hard/Soft Iron; C 3x3 + B 3x1) */
    VN100_REG_REF_FRAME_ROTATION= 26,  /* Reg 26 - Reference Frame Rotation */
    VN100_REG_VPE_BASIC_CONTROL = 35,  /* Reg 35 - VPE Basic Control */
    VN100_REG_HSI_CONTROL       = 44,  /* Reg 44 - Magnetometer Calibration Control (HSIMode/Output/Rate) */
    /* WARNING: Reg 46 does **NOT EXIST** on FW 3.1.0.0 (ICD discrepancy #1).
       This constant exists only for FW v2.x backward compatibility; new code
       must NOT rely on it - convergence is measured off Reg 47 stability
       instead (pyvn100/capabilities.py: has_hsi_status_reg). */
    VN100_REG_HSI_STATUS        = 46,  /* Reg 46 - Magnetometer Calibration Status (read-only; FW v2.x ONLY) */
    VN100_REG_HSI_CALCULATED    = 47,  /* Reg 47 - Calculated Magnetometer Calibration (read-only) */
    VN100_REG_BINARY_OUTPUT_1   = 75,  /* Reg 75 - Binary Output 1 (AsyncMode/RateDivisor/Group/Field) */
    VN100_REG_BINARY_OUTPUT_2   = 76,  /* Reg 76 - Binary Output 2 */
    VN100_REG_BINARY_OUTPUT_3   = 77,  /* Reg 77 - Binary Output 3 */
    VN100_REG_GYRO_COMPENSATION = 84   /* Reg 84 - Gyro Compensation (C 3x3 + B 3x1; includes bias) */
};

/* ADOR (Reg 6) values (UM001 Table 28) */
enum { VN100_ADOR_OFF = 0, VN100_ADOR_VNYMR = 14 };

/* Binary Output (Reg 75-77) - output Hz = VN100_IMU_RATE_HZ / RateDivisor */
enum { VN100_IMU_RATE_HZ = 800 };
#define VN100_BIN_GROUP_COMMON          0x01u   /* OutputGroup: Common only */
#define VN100_BIN_FIELDS_YPR_RATE_ACCEL 0x0128u /* OutputField: YawPitchRoll(3)+AngularRate(5)+Accel(8) */

/* Reg 75-77 - AsyncMode (UM001 section 4.2) */
enum { VN100_ASYNC_OFF = 0, VN100_ASYNC_PORT1 = 1, VN100_ASYNC_PORT2 = 2, VN100_ASYNC_BOTH = 3 };

/* Serial port the sensor is wired to (this project: Rugged pin 8/9 = TTL Serial Port 2). */
#define VN100_SENSOR_ASYNC_PORT VN100_ASYNC_PORT2

/* Reg 44 - HSIMode (UM001 Table 2) */
enum { VN100_HSI_OFF = 0, VN100_HSI_RUN = 1, VN100_HSI_RESET = 2 };

/* Reg 44 - HSIOutput (UM001 Table 3): two valid values.
   NO_ONBOARD -> onboard compensation not applied (output is raw Reg 23, or
   identity if uncalibrated)
   USE_ONBOARD -> onboard real-time HSI (Reg 47) is applied */
enum { VN100_HSI_OUT_NO_ONBOARD = 1, VN100_HSI_OUT_USE_ONBOARD = 3 };

/* Reg 35 - HeadingMode */
enum { VN100_HEADING_ABSOLUTE = 0, VN100_HEADING_RELATIVE = 1, VN100_HEADING_INDOOR = 2 };

#ifdef __cplusplus
}
#endif

#endif /* VN100_REGISTERS_H */
