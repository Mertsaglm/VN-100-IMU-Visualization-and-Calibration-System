/**
  ******************************************************************************
  * @file    vn100_protocol.h
  * @brief   VN-100 protocol layer (MID level, platform-INDEPENDENT)
  ******************************************************************************
  * C-side counterpart of the docs/protocol.md contract. Behaves identically
  * to pyvn100.protocol (same test vectors: xor("VNYMR")=0x5E,
  * crc16("123456789")=0x31C3).
  ******************************************************************************
  */
#ifndef VN100_PROTOCOL_H
#define VN100_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "vn100_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* NOTE: the binary sync byte has a SINGLE definition, VN100_BIN_SYNC in
   vn100_binary.h; no second definition is kept here. */

/** XOR checksum (8-bit) over the bytes between '$' and '*'. */
uint8_t vn100_xor_checksum(const char *data, uint16_t len);

/** Validates the checksum of an ASCII message. */
bool vn100_ascii_checksum_ok(const char *msg, uint16_t len);

/** Parses a $VNYMR message. Returns VN100_ERR_* if invalid. */
vn100_status_t vn100_parse_vnymr(const char *msg, uint16_t len, vn100_data_t *out);

/**
 * Encodes a measurement into a full "$VNYMR,...*CS\r\n" line (inverse of
 * parse). The STM32 ground-station bridge re-publishes sensor data to the PC
 * (VCP) in this format, so the dashboard sees a clean $VNYMR regardless of
 * the active mode (ASCII/binary). Returns bytes written to buf (<=0 on error).
 * @note requires "float with printf" (newlib-nano) enabled for %f output.
 */
int vn100_encode_vnymr(char *buf, int bufsize, const vn100_data_t *d);

/** VectorNav binary CRC-16 (CCITT, init=0). */
uint16_t vn100_crc16(const uint8_t *data, uint16_t len);

/* -- Command builders: write "$...*CS\r\n" to buf, return byte count (<=0 on error) -- */
int vn100_cmd_read_register(char *buf, int bufsize, uint16_t reg);
int vn100_cmd_write_register_u32(char *buf, int bufsize, uint16_t reg, uint32_t val);
int vn100_cmd_write_register_floats(char *buf, int bufsize, uint16_t reg, const float *vals, int n);
int vn100_cmd_simple(char *buf, int bufsize, const char *mnemonic); /* "VNTAR","VNWNV","VNSGB",... */

/** Builds a Binary Output register (75-77) config command: "$VNWRG,75,<am>,<rd>,<GG>,<FFFF>*CS".
    OutputGroup/OutputField are written in HEX (docs/protocol.md, section 4.2). */
int vn100_cmd_binary_output(char *buf, int bufsize, uint16_t reg,
                            uint16_t async_mode, uint16_t rate_divisor,
                            uint8_t group, uint16_t fields);

/** Self-test: does the protocol layer produce the reference vectors? (no hardware required) */
bool vn100_protocol_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* VN100_PROTOCOL_H */
