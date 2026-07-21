/**
  ******************************************************************************
  * @file    vn100_protocol.c
  * @brief   VN-100 protocol layer implementation (MID level, HAL-free)
  ******************************************************************************
  */
#include "vn100_protocol.h"

#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* -- Checksum ----------------------------------------------------- */

uint8_t vn100_xor_checksum(const char *data, uint16_t len)
{
    uint8_t cs = 0U;
    uint16_t i;
    for (i = 0U; i < len; i++)
    {
        cs ^= (uint8_t)data[i];
    }
    return cs;
}

bool vn100_ascii_checksum_ok(const char *msg, uint16_t len)
{
    uint16_t star = 0U;
    uint16_t i;
    uint8_t  calc, recv;
    char     hex[3];

    for (i = 0U; i < len; i++)
    {
        if (msg[i] == '*')
        {
            star = i;
            break;
        }
    }
    if (star == 0U)
    {
        return false;                      /* no '*', or '*' is the first character */
    }
    if ((uint16_t)(star + 2U) >= len)
    {
        return false;                      /* checksum digits missing */
    }

    calc = vn100_xor_checksum(&msg[1], (uint16_t)(star - 1U));
    hex[0] = msg[star + 1U];
    hex[1] = msg[star + 2U];
    hex[2] = '\0';
    /* Parity with the Python side: strtol doesn't validate hex digits, so a
       malformed checksum like "*GG" could parse as 0 and pass. Python
       rejects this; C must reject it too. */
    if (!isxdigit((unsigned char)hex[0]) || !isxdigit((unsigned char)hex[1]))
    {
        return false;
    }
    recv = (uint8_t)strtol(hex, NULL, 16);
    return (calc == recv);
}

/* -- $VNYMR parsing ------------------------------------------------ */

static int parse_floats(const char *p, float *f, int maxn)
{
    int n = 0;
    char *end;
    while ((n < maxn) && (*p != '\0') && (*p != '*'))
    {
        f[n] = strtof(p, &end);
        if (end == p)
        {
            break;                         /* no progress -> parse failed */
        }
        n++;
        p = end;
        if (*p == ',')
        {
            p++;
        }
    }
    return n;
}

vn100_status_t vn100_parse_vnymr(const char *msg, uint16_t len, vn100_data_t *out)
{
    float f[12];

    if ((out == NULL) || (len < 20U))
    {
        return VN100_ERR_PARSE;
    }
    if (msg[0] != '$')
    {
        return VN100_ERR_PARSE;
    }
    if (memcmp(msg, "$VNYMR", 6) != 0)
    {
        return VN100_ERR_PARSE;            /* different message type */
    }
    if (msg[6] != ',')
    {
        return VN100_ERR_PARSE;            /* parity with Python: reject an extended header ("$VNYMRX1,..") */
    }
    if (!vn100_ascii_checksum_ok(msg, len))
    {
        return VN100_ERR_CRC;
    }

    /* "$VNYMR," = 7 characters */
    if (parse_floats(msg + 7, f, 12) != 12)
    {
        return VN100_ERR_PARSE;
    }

    out->yaw = f[0];   out->pitch = f[1];  out->roll = f[2];
    out->mag_x = f[3]; out->mag_y = f[4];  out->mag_z = f[5];
    out->accel_x = f[6]; out->accel_y = f[7]; out->accel_z = f[8];
    out->gyro_x = f[9];  out->gyro_y = f[10]; out->gyro_z = f[11];
    out->timestamp = 0U;
    return VN100_OK;
}

/* -- $VNYMR encoding (inverse of parse - ground-station relay) --- */

/* Defined below in the "Command builders" section; forward declaration. */
static int wrap_command(char *buf, int bufsize, const char *body);

int vn100_encode_vnymr(char *buf, int bufsize, const vn100_data_t *d)
{
    /* Field precisions match the Python simulator (ascii_frame) exactly:
       angles/accel %.3f, mag/gyro %.4f -> both sides round-trip test against each other. */
    char body[128];
    int  bl;

    if ((buf == NULL) || (d == NULL))
    {
        return -1;
    }
    /* If mag is unknown (source was a binary frame), don't produce a
       $VNYMR: the 12-field format can't "leave mag blank", and writing
       0.0000 would publish a fake measurement with a VALID CHECKSUM
       indistinguishable from real data. The caller (vn_relay) counts -1
       returns (g_relay_encode_fail) so the loss is VISIBLE, not a silent
       lie. */
    if (d->mag_missing != 0U)
    {
        return -1;
    }
    bl = snprintf(body, sizeof(body),
                  "VNYMR,%+.3f,%+.3f,%+.3f,%+.4f,%+.4f,%+.4f,"
                  "%+.3f,%+.3f,%+.3f,%+.4f,%+.4f,%+.4f",
                  (double)d->yaw,     (double)d->pitch,   (double)d->roll,
                  (double)d->mag_x,   (double)d->mag_y,   (double)d->mag_z,
                  (double)d->accel_x, (double)d->accel_y, (double)d->accel_z,
                  (double)d->gyro_x,  (double)d->gyro_y,  (double)d->gyro_z);
    if ((bl <= 0) || (bl >= (int)sizeof(body)))
    {
        return -1;
    }
    return wrap_command(buf, bufsize, body);
}

/* -- Binary CRC-16 (VectorNav, init=0) ---------------------------- */

uint16_t vn100_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0U;
    uint16_t i;
    for (i = 0U; i < len; i++)
    {
        crc = (uint16_t)((crc >> 8) | (crc << 8));
        crc ^= (uint16_t)data[i];
        crc ^= (uint16_t)((crc & 0xFFU) >> 4);
        crc ^= (uint16_t)(crc << 12);
        crc ^= (uint16_t)((crc & 0x00FFU) << 5);
    }
    return crc;
}

/* -- Command builders ---------------------------------------------- */

static int wrap_command(char *buf, int bufsize, const char *body)
{
    uint8_t cs = vn100_xor_checksum(body, (uint16_t)strlen(body));
    int n = snprintf(buf, (size_t)bufsize, "$%s*%02X\r\n", body, (unsigned)cs);
    return ((n > 0) && (n < bufsize)) ? n : -1;
}

int vn100_cmd_read_register(char *buf, int bufsize, uint16_t reg)
{
    char body[24];
    int bl = snprintf(body, sizeof(body), "VNRRG,%u", (unsigned)reg);
    if ((bl <= 0) || (bl >= (int)sizeof(body)))
    {
        return -1;
    }
    return wrap_command(buf, bufsize, body);
}

int vn100_cmd_write_register_u32(char *buf, int bufsize, uint16_t reg, uint32_t val)
{
    char body[40];
    int bl = snprintf(body, sizeof(body), "VNWRG,%u,%lu", (unsigned)reg, (unsigned long)val);
    if ((bl <= 0) || (bl >= (int)sizeof(body)))
    {
        return -1;
    }
    return wrap_command(buf, bufsize, body);
}

int vn100_cmd_write_register_floats(char *buf, int bufsize, uint16_t reg, const float *vals, int n)
{
    char body[256];   /* 12 floats (Reg 23/84) can exceed 128 bytes at realistic values and get the command rejected */
    int off = snprintf(body, sizeof(body), "VNWRG,%u", (unsigned)reg);
    int i;

    if ((off <= 0) || (off >= (int)sizeof(body)) || (vals == NULL))
    {
        return -1;
    }
    for (i = 0; i < n; i++)
    {
        int w = snprintf(body + off, sizeof(body) - (size_t)off, ",%.6f", (double)vals[i]);
        if ((w <= 0) || (w >= (int)(sizeof(body) - (size_t)off)))
        {
            return -1;   /* overflow */
        }
        off += w;
    }
    return wrap_command(buf, bufsize, body);
}

int vn100_cmd_simple(char *buf, int bufsize, const char *mnemonic)
{
    return wrap_command(buf, bufsize, mnemonic);
}

int vn100_cmd_binary_output(char *buf, int bufsize, uint16_t reg,
                            uint16_t async_mode, uint16_t rate_divisor,
                            uint8_t group, uint16_t fields)
{
    char body[48];
    int bl = snprintf(body, sizeof(body), "VNWRG,%u,%u,%u,%02X,%04X",
                      (unsigned)reg, (unsigned)async_mode, (unsigned)rate_divisor,
                      (unsigned)group, (unsigned)fields);
    if ((bl <= 0) || (bl >= (int)sizeof(body)))
    {
        return -1;
    }
    return wrap_command(buf, bufsize, body);
}

/* -- Self-test ------------------------------------------------------ */

bool vn100_protocol_selftest(void)
{
    if (vn100_xor_checksum("VNYMR", 5) != 0x5EU)
    {
        return false;
    }
    if (vn100_crc16((const uint8_t *)"123456789", 9) != 0x31C3U)
    {
        return false;
    }
    return true;
}
