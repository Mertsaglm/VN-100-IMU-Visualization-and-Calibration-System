/**
  ******************************************************************************
  * @file    vn100_types.h
  * @brief   VN-100 common data structures (platform-INDEPENDENT)
  ******************************************************************************
  * Field-for-field match with pyvn100.types.Vn100Data on the Python side.
  ******************************************************************************
  */
#ifndef VN100_TYPES_H
#define VN100_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Driver status codes */
typedef enum
{
    VN100_OK        = 0,   /**< Success                         */
    VN100_ERR       = 1,   /**< General error                   */
    VN100_ERR_PARAM = 2,   /**< Invalid parameter                */
    VN100_ERR_CRC   = 3,   /**< Checksum/CRC error               */
    VN100_ERR_PARSE = 4    /**< Message parse error              */
    /* There is NO separate error code for "no data": that case is conveyed
       via the bool return of vn100_get_data. */
} vn100_status_t;

/** A single VN-100 measurement */
typedef struct
{
    float yaw, pitch, roll;            /**< Euler angles [deg]       */
    float mag_x, mag_y, mag_z;         /**< Magnetic field [Gauss]   */
    float accel_x, accel_y, accel_z;   /**< Acceleration [m/s^2]     */
    float gyro_x, gyro_y, gyro_z;      /**< Angular rate [rad/s]     */
    uint32_t timestamp;                /**< Time received [ms]       */
    /** The binary frame carries no magnetometer field (see docs/protocol.md).
        0 = mag valid (DEFAULT; a memset struct stays correct on the ASCII
        path), 1 = mag unknown. `vn100_encode_vnymr` skips the frame while
        set: otherwise a $VNYMR with mag=0.0000 and a VALID CHECKSUM would
        go out, indistinguishable from real data (silent bad data). */
    uint8_t mag_missing;
} vn100_data_t;

#ifdef __cplusplus
}
#endif

#endif /* VN100_TYPES_H */
