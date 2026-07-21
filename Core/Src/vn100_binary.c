/**
  ******************************************************************************
  * @file    vn100_binary.c
  * @brief   VN-100 binary protocol codec implementation (HAL-free)
  ******************************************************************************
  * The Cortex-M7 is little-endian, so float fields are copied directly via
  * memcpy. CRC-16 comes from the protocol layer (vn100_crc16).
  ******************************************************************************
  */
#include "vn100_binary.h"
#include "vn100_protocol.h"

#include <string.h>

#define BIN_GROUPS     0x01u
#define BIN_FIELDS_G1  0x0128u   /* (1<<3)|(1<<5)|(1<<8): YPR, AngularRate, Accel */
#define BIN_PAYLOAD    36        /* 9 × float32 */

vn100_status_t vn100_binary_decode(const uint8_t *f, uint16_t len, vn100_data_t *out)
{
    float v[9];
    uint16_t fields;

    if ((out == NULL) || (len < VN100_BIN_FRAME_LEN))
    {
        return VN100_ERR_PARSE;
    }
    if (f[0] != VN100_BIN_SYNC)
    {
        return VN100_ERR_PARSE;
    }
    if (f[1] != BIN_GROUPS)
    {
        return VN100_ERR_PARSE;
    }
    fields = (uint16_t)((uint16_t)f[2] | ((uint16_t)f[3] << 8));
    if (fields != BIN_FIELDS_G1)
    {
        return VN100_ERR_PARSE;
    }
    /* CRC over the whole body including the CRC field itself (sync excluded) must equal 0 */
    if (vn100_crc16(&f[1], (uint16_t)(VN100_BIN_FRAME_LEN - 1)) != 0U)
    {
        return VN100_ERR_CRC;
    }

    memcpy(v, &f[4], BIN_PAYLOAD);
    out->yaw = v[0];    out->pitch = v[1];   out->roll = v[2];
    out->gyro_x = v[3]; out->gyro_y = v[4];  out->gyro_z = v[5];
    out->accel_x = v[6]; out->accel_y = v[7]; out->accel_z = v[8];
    /* Binary frame carries NO magnetometer field. Zeroed for a clean struct,
       but these zeros are NOT a measurement -> flagged so the ASCII relay
       doesn't publish a fake $VNYMR with mag=0.0000. */
    out->mag_x = 0.0f;  out->mag_y = 0.0f;   out->mag_z = 0.0f;
    out->mag_missing = 1U;
    out->timestamp = 0U;
    return VN100_OK;
}

int vn100_binary_encode(uint8_t *buf, int bufsize, const vn100_data_t *d)
{
    float v[9];
    uint16_t crc;

    if ((buf == NULL) || (d == NULL) || (bufsize < VN100_BIN_FRAME_LEN))
    {
        return -1;
    }

    buf[0] = VN100_BIN_SYNC;
    buf[1] = BIN_GROUPS;
    buf[2] = (uint8_t)(BIN_FIELDS_G1 & 0xFFu);
    buf[3] = (uint8_t)(BIN_FIELDS_G1 >> 8);

    v[0] = d->yaw;    v[1] = d->pitch;   v[2] = d->roll;
    v[3] = d->gyro_x; v[4] = d->gyro_y;  v[5] = d->gyro_z;
    v[6] = d->accel_x; v[7] = d->accel_y; v[8] = d->accel_z;
    memcpy(&buf[4], v, BIN_PAYLOAD);

    /* CRC over groups+fields+payload = 39 bytes (sync excluded, through end of payload) */
    crc = vn100_crc16(&buf[1], (uint16_t)(3 + BIN_PAYLOAD));
    buf[40] = (uint8_t)(crc >> 8);      /* big-endian */
    buf[41] = (uint8_t)(crc & 0xFFu);
    return VN100_BIN_FRAME_LEN;
}

bool vn100_binary_selftest(void)
{
    vn100_data_t a, b;
    uint8_t buf[VN100_BIN_FRAME_LEN];

    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    a.yaw = 10.0f; a.pitch = -5.0f; a.roll = 3.0f;
    a.accel_z = 9.81f; a.gyro_x = 0.5f;

    if (vn100_binary_encode(buf, sizeof(buf), &a) != VN100_BIN_FRAME_LEN)
    {
        return false;
    }
    if (vn100_binary_decode(buf, sizeof(buf), &b) != VN100_OK)
    {
        return false;
    }
    return (b.yaw == a.yaw) && (b.pitch == a.pitch) &&
           (b.accel_z == a.accel_z) && (b.gyro_x == a.gyro_x);
}
