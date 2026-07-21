/**
  ******************************************************************************
  * @file    vn100_binary.h
  * @brief   VN-100 binary protocol codec (MID level, platform-INDEPENDENT)
  ******************************************************************************
  * Byte-for-byte the same layout as pyvn100.binary (docs/protocol.md,
  * section 3):
  *   0xFA | groups(0x01) | fieldMask(LE,0x0128) | 9x float32(LE) | CRC16(BE)
  *   Fields: YawPitchRoll + AngularRate + Accel = 36-byte payload, 42 bytes total
  ******************************************************************************
  */
#ifndef VN100_BINARY_H
#define VN100_BINARY_H

#include <stdint.h>
#include <stdbool.h>
#include "vn100_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VN100_BIN_SYNC       0xFAu
#define VN100_BIN_FRAME_LEN  42

/** Decodes a 42-byte binary frame. */
vn100_status_t vn100_binary_decode(const uint8_t *frame, uint16_t len, vn100_data_t *out);

/** Encodes a vn100_data_t into a 42-byte binary frame (buf >= 42). Returns byte count, or -1. */
int vn100_binary_encode(uint8_t *buf, int bufsize, const vn100_data_t *d);

/** Self-test: encode->decode round-trip (no hardware required). */
bool vn100_binary_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* VN100_BINARY_H */
