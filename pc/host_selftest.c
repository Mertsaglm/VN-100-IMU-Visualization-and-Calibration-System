/**
 ******************************************************************************
 * @file    host_selftest.c
 * @brief   Compiles and verifies the portable C core (protocol layer) on
 *          PC — proof of the "same core on PC + STM32" principle.
 ******************************************************************************
 * Builds the HAL-free portable layers (vn100_protocol.c + vn100.c +
 * vn100_binary.c) with no STM32 dependency, testing protocol/core behavior
 * under a real compiler without hardware — verified bit-for-bit identical
 * to Python (pyvn100).
 *
 * Build & run (easy way: `cd pc && make selftest`):
 *   gcc -std=c11 -Wall -Wextra -I Core/Inc pc/host_selftest.c \
 *       Core/Src/vn100_protocol.c Core/Src/vn100.c Core/Src/vn100_binary.c \
 *       -o build/host_selftest -lm
 *   ./build/host_selftest
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "vn100_protocol.h"
#include "vn100_binary.h"
#include "vn100.h"
#include "host_link.h"

static int g_fail = 0;

static void check(int cond, const char *name)
{
    printf("  [%s] %s\n", cond ? "OK " : "FAIL", name);
    if (!cond)
    {
        g_fail = 1;
    }
}

/* For host_link tests: captures replies to the PC into a buffer */
static char g_reply[256];
static int hl_reply(void *ctx, const uint8_t *data, uint16_t len)
{
    size_t cur = strlen(g_reply);
    (void)ctx;
    if ((cur + (size_t)len) < sizeof(g_reply))
    {
        memcpy(g_reply + cur, data, len);
        g_reply[cur + len] = '\0';
    }
    return (int)len;
}
/* Captures bytes sent TO THE SENSOR, not the reply to the PC — a wrong
   register could be written while still returning VNACK. */
static char g_sensor[512];
static int capture_write(void *ctx, const uint8_t *data, uint16_t len)
{
    size_t cur = strlen(g_sensor);
    (void)ctx;
    if ((cur + (size_t)len) < sizeof(g_sensor))
    {
        memcpy(g_sensor + cur, data, len);
        g_sensor[cur + len] = '\0';
    }
    return (int)len;
}

int main(void)
{
    printf("VN-100 portable core - PC self-test\n");

    /* 1) The core's own self-test (shared checksum + CRC vectors) */
    check(vn100_protocol_selftest(), "vn100_protocol_selftest (shared xor/crc vectors)");

    /* 1b) main.c also runs vn100_binary_selftest at board boot; if skipped
          here, a break in the binary codec's self-test would surface only
          on the board, not on PC. */
    check(vn100_binary_selftest(), "vn100_binary_selftest (binary codec self-test)");

    /* 2) Known checksum/CRC vectors (identical to Python) */
    check(vn100_xor_checksum("VNYMR", 5) == 0x5E, "xor(\"VNYMR\") == 0x5E");
    check(vn100_crc16((const uint8_t *)"123456789", 9) == 0x31C3, "crc16(\"123456789\") == 0x31C3");

    /* 3) $VNYMR encode -> parse round-trip (verifies the relay path) */
    {
        vn100_data_t in;
        vn100_data_t out;
        char line[160];
        int  n;

        memset(&in, 0, sizeof(in));
        memset(&out, 0, sizeof(out));
        in.yaw = 12.500f;   in.pitch = -3.250f;  in.roll = 90.000f;
        in.mag_x = 0.2280f; in.mag_y = -0.0150f; in.mag_z = -0.3870f;
        in.accel_x = 0.100f; in.accel_y = -0.200f; in.accel_z = 9.810f;
        in.gyro_x = 0.0010f; in.gyro_y = -0.0020f; in.gyro_z = 0.0030f;

        n = vn100_encode_vnymr(line, sizeof(line), &in);
        check(n > 0, "vn100_encode_vnymr produced bytes");

        check((n >= 2) && (line[n - 2] == '\r') && (line[n - 1] == '\n'),
              "encode output ends with \\r\\n");
        check(strncmp(line, "$VNYMR,", 7) == 0, "encode output starts with $VNYMR");

        check(vn100_parse_vnymr(line, (uint16_t)n, &out) == VN100_OK,
              "encoded line can be parsed back");

        check(fabsf(out.yaw - in.yaw)     < 0.01f, "round-trip: yaw preserved");
        check(fabsf(out.pitch - in.pitch) < 0.01f, "round-trip: pitch preserved");
        check(fabsf(out.roll - in.roll)   < 0.01f, "round-trip: roll preserved");
        check(fabsf(out.mag_x - in.mag_x) < 0.001f, "round-trip: mag_x preserved");
        check(fabsf(out.mag_z - in.mag_z) < 0.001f, "round-trip: mag_z preserved");
        check(fabsf(out.accel_z - in.accel_z) < 0.01f, "round-trip: accel_z preserved");
        check(fabsf(out.gyro_z - in.gyro_z)   < 0.001f, "round-trip: gyro_z preserved");

        /* 4) Format identical to the Python simulator (ascii_frame):
              same values, same precision -> same body string. */
        {
            const char *expect =
                "$VNYMR,+12.500,-3.250,+90.000,"
                "+0.2280,-0.0150,-0.3870,"
                "+0.100,-0.200,+9.810,"
                "+0.0010,-0.0020,+0.0030";
            check(strncmp(line, expect, strlen(expect)) == 0,
                  "encode body identical to Python ascii_frame");
            /* Counterpart: tests/test_wire_format.py::test_ascii_encode_C_cekirdegi_ile_BIREBIR
               (_YMR_GOLDEN) - if one drifts, the other BREAKS. */
        }

        /* 4b) Float format (%.6f) of the Reg 23 calibration write - must be
              IDENTICAL to Python protocol.write_register. Drifting to `%g`
              produces "5e-05"; the sensor rejects it, and without this test
              nothing else would catch it.
              Counterpart: tests/test_wire_format.py::test_reg23_float_bicimi_... */
        {
            static const float cal12[12] = {
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f,
                0.00005f, -0.08f, 0.03f
            };
            char cbuf[256];
            const char *cexp =
                "$VNWRG,23,1.000000,0.000000,0.000000,"
                "0.000000,1.000000,0.000000,"
                "0.000000,0.000000,1.000000,"
                "0.000050,-0.080000,0.030000";
            int cn = vn100_cmd_write_register_floats(cbuf, (int)sizeof(cbuf), 23, cal12, 12);
            check(cn > 0, "Reg 23 float command generated");
            check(strncmp(cbuf, cexp, strlen(cexp)) == 0,
                  "Reg 23 float format (%.6f) identical to Python");
            check(strchr(cbuf, 'e') == NULL && strchr(cbuf, 'E') == NULL,
                  "Reg 23 write has NO scientific notation (no %g drift)");
        }

        /* C<->Python parity: a corrupt checksum digit and an extended header
           MUST BE REJECTED - an unvalidated strtol could read "*GG" as 0,
           and a fixed msg+7 offset could accept an extended header like
           "$VNYMRX,..". */
        {
            const char *bad_hex =
                "$VNYMR,+1.0,+2.0,+3.0,+0.1,+0.2,+0.3,+0.0,+0.0,+9.8,+0.0,+0.0,+0.0*GG";
            const char *bad_hdr =
                "$VNYMRX,+1.0,+2.0,+3.0,+0.1,+0.2,+0.3,+0.0,+0.0,+9.8,+0.0,+0.0,+0.0*00";
            check(vn100_parse_vnymr(bad_hex, (uint16_t)strlen(bad_hex), &out) == VN100_ERR_CRC,
                  "parser: non-hex checksum ('*GG') is REJECTED (same behavior as Python)");
            check(vn100_parse_vnymr(bad_hdr, (uint16_t)strlen(bad_hdr), &out) == VN100_ERR_PARSE,
                  "parser: extended header ('$VNYMRX,') is REJECTED (same behavior as Python)");
        }
    }

    /* The binary->ASCII relay must NOT fabricate mag data. The binary frame
       has NO mag field; writing 0.0000 would publish a made-up reading
       with a VALID CHECKSUM. */
    {
        vn100_data_t bd;
        char line2[160];
        memset(&bd, 0, sizeof(bd));
        bd.yaw = 1.0f; bd.accel_z = 9.81f;
        check(vn100_encode_vnymr(line2, (int)sizeof(line2), &bd) > 0,
              "encode: frame produced when mag IS valid (default path intact)");
        bd.mag_missing = 1U;                  /* binary decode flags this */
        check(vn100_encode_vnymr(line2, (int)sizeof(line2), &bd) < 0,
              "encode: $VNYMR NOT produced when mag missing (no fake 0.0000 published)");
    }

    /* REGRESSION: losing a single byte in BINARY mode shifts the frame; the
       parser still unconditionally consumes 42 bytes, pulling in the command
       reply that follows. If a CRC mismatch then discards all 42 bytes, the
       $VNRRG/$VNERR that is the only proof of "read back to verify" would be
       silently lost. Instead, parsing resumes as ASCII from the '$' inside
       the corrupt frame, recovering the reply. */
    {
        vn100_t v;
        vn100_port_t port;
        char resp[VN100_RESP_MAX];
        uint16_t rlen = 0;
        uint8_t frame[VN100_BIN_FRAME_LEN];
        vn100_data_t src;
        const char *rrg = "$VNRRG,23,1.5,0,0,0,1,0,0,0,1,0.1,0.2,0.3*57\r\n";

        memset(&port, 0, sizeof(port));
        vn100_init(&v, &port, NULL, NULL, NULL);

        memset(&src, 0, sizeof(src));
        src.yaw = 1.0f; src.accel_z = 9.81f;
        check(vn100_binary_encode(frame, (int)sizeof(frame), &src) == VN100_BIN_FRAME_LEN,
              "M-2: source binary frame produced");

        /* Simulate byte loss: drop the frame's last byte, so the parser is
           one byte short and starts pulling the following ASCII reply in. */
        vn100_rx_feed(&v, frame, VN100_BIN_FRAME_LEN - 1U);
        vn100_rx_feed(&v, (const uint8_t *)rrg, (uint16_t)strlen(rrg));

        check(v.error_count >= 1U, "M-2: corrupt frame counted as an ERROR");
        check(vn100_take_response(&v, resp, sizeof(resp), &rlen),
              "M-2: swallowed command reply was RECOVERED (landed in the mailbox)");
        check((rlen > 0U) && (strncmp(resp, "$VNRRG,23", 9) == 0),
              "M-2: recovered reply is the correct line");
    }

    /* 5) Core routing: $VNYMR -> telemetry, $VN... -> reply mailbox */
    {
        vn100_t v;
        vn100_port_t port;
        char resp[VN100_RESP_MAX];
        uint16_t rlen = 0;
        const char *ymr = "$VNYMR,+1.0,+2.0,+3.0,+0.1,+0.2,+0.3,+0.0,+0.0,+9.8,+0.0,+0.0,+0.0*5F\r\n";
        const char *rrg = "$VNRRG,46,3,120,0.0123,0.1,0.2,0.3,10,12,9,11,8,10,13,7*76\r\n";

        memset(&port, 0, sizeof(port));   /* write/millis/crit = NULL -> core is NULL-safe */
        check(vn100_init(&v, &port, NULL, NULL, NULL) == VN100_OK, "vn100_init");

        /* When $VNYMR is fed in: counted as telemetry, reply mailbox stays EMPTY */
        vn100_rx_feed(&v, (const uint8_t *)ymr, (uint16_t)strlen(ymr));
        check(v.packet_count == 1U, "core: $VNYMR counted as telemetry");
        check(!vn100_take_response(&v, resp, sizeof(resp), &rlen),
              "core: $VNYMR did NOT land in the reply mailbox");

        /* When $VNRRG is fed in: lands verbatim in the reply mailbox */
        vn100_rx_feed(&v, (const uint8_t *)rrg, (uint16_t)strlen(rrg));
        check(v.packet_count == 1U, "core: $VNRRG not counted as telemetry");
        check(vn100_take_response(&v, resp, sizeof(resp), &rlen),
              "core: $VNRRG landed in the reply mailbox");
        check(strncmp(resp, "$VNRRG,46,", 10) == 0, "core: reply delivered verbatim");
        check(!vn100_take_response(&v, resp, sizeof(resp), &rlen),
              "core: mailbox cleared after being read");
    }

    /* 6) DUAL-MODE parser: binary frame + mixed (binary + ASCII reply) stream */
    {
        vn100_t v;
        vn100_port_t port;
        vn100_data_t in;
        uint8_t frame[VN100_BIN_FRAME_LEN];
        const char *rrg = "$VNRRG,46,3,120,0.0123,0.1,0.2,0.3,10,12,9,11,8,10,13,7*76\r\n";
        char resp[VN100_RESP_MAX];
        uint16_t rlen = 0;

        memset(&port, 0, sizeof(port));
        memset(&in, 0, sizeof(in));
        in.yaw = 10.0f; in.pitch = -5.0f; in.roll = 3.0f;
        in.gyro_x = 0.01f; in.accel_z = 9.81f;
        check(vn100_binary_encode(frame, sizeof(frame), &in) == VN100_BIN_FRAME_LEN,
              "binary encode produced 42 bytes");

        /* WIRE-FORMAT spec anchor (independent of decode - matches Python
           test_wire_format): 0xFA | groups=0x01 | fieldMask=0x0128
           (LE -> 28 01) | ... | CRC(BE) */
        check((frame[0] == 0xFAu) && (frame[1] == 0x01u) &&
              (frame[2] == 0x28u) && (frame[3] == 0x01u),
              "binary header bytes match the spec (FA 01 28 01)");
        check(vn100_crc16(&frame[1], (uint16_t)(VN100_BIN_FRAME_LEN - 1)) == 0U,
              "binary CRC big-endian: over the packet (after 0xFA) == 0");

        vn100_init(&v, &port, NULL, NULL, NULL);

        /* A single binary frame -> decoded as telemetry */
        vn100_rx_feed(&v, frame, VN100_BIN_FRAME_LEN);
        check(v.packet_count == 1U, "core: binary frame counted as telemetry");
        check(fabsf(v.data.accel_z - 9.81f) < 0.001f, "core: binary accel_z decoded");

        /* MIXED stream: binary + ASCII command reply + binary */
        vn100_rx_feed(&v, frame, VN100_BIN_FRAME_LEN);
        vn100_rx_feed(&v, (const uint8_t *)rrg, (uint16_t)strlen(rrg));
        vn100_rx_feed(&v, frame, VN100_BIN_FRAME_LEN);
        check(v.packet_count == 3U, "core: 3 binary telemetry frames (2+1) decoded in mixed stream");
        check(vn100_take_response(&v, resp, sizeof(resp), &rlen),
              "core: ASCII $VNRRG reply also caught in mixed stream");
        check(strncmp(resp, "$VNRRG,46,", 10) == 0, "core: mixed-stream reply verbatim");
    }

    /* 7) host_link: 'VN MODE' sets the RELAY format and returns an ack */
    {
        vn100_t v;
        vn100_port_t port;
        host_link_t hl;

        memset(&port, 0, sizeof(port));
        port.write = capture_write;              /* treat sensor TX as successful + CAPTURE it */
        vn100_init(&v, &port, NULL, NULL, NULL);
        host_link_init(&hl, &v, hl_reply, NULL);
        check(hl.out_fmt == VN100_FMT_ASCII, "host_link: default relay format is ASCII");

        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN MODE BINARY\n", 15U);
        host_link_process(&hl);
        check(hl.out_fmt == VN100_FMT_BINARY, "host_link: 'VN MODE BINARY' -> relay BINARY");
        check(strstr(g_reply, "VNMODE BINARY") != NULL, "host_link: BINARY ack returned");

        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN MODE ASCII\n", 14U);
        host_link_process(&hl);
        check(hl.out_fmt == VN100_FMT_ASCII, "host_link: 'VN MODE ASCII' -> relay ASCII");
        check(strstr(g_reply, "VNMODE ASCII") != NULL, "host_link: ASCII ack returned");

        /* 7b) FREQ/TYPE/MODE argument validation tests */
        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN FREQ 300\n", 12U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR freq-range") != NULL,
              "host_link: 'VN FREQ 300' out of range -> VNERR (NOT rounded to 400 Hz)");
        check(hl.last_hz == 0U, "host_link: rejected FREQ does NOT change last_hz");

        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN FREQ 10\n", 11U);
        host_link_process(&hl);
        check(hl.last_hz == 10U, "host_link: valid 'VN FREQ 10' sets last_hz=10");

        /* REGRESSION: in ASCII mode, 'VN FREQ 200' must clamp ADOF to 50, not
           write 200. If the clamp only lived in the MODE branch, FREQ would
           stay unclamped and 200 Hz would blow the band ceiling (~98-114 Hz)
           2x over. */
        check(hl.out_fmt == VN100_FMT_ASCII, "host_link: (precondition) mode is ASCII");
        g_sensor[0] = '\0';
        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN FREQ 200\n", 12U);
        host_link_process(&hl);
        check(strstr(g_sensor, "$VNWRG,7,50") != NULL,
              "host_link: in ASCII, 'VN FREQ 200' CLAMPS ADOF to 50");
        check(strstr(g_sensor, "$VNWRG,7,200") == NULL,
              "host_link: in ASCII, ADOF 200 is NOT written (band ceiling not 2x exceeded)");
        /* last_hz stores the RAW value -> switching to BINARY restores 200. */
        check(hl.last_hz == 200U, "host_link: clamp does NOT overwrite last_hz");

        g_sensor[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN MODE BINARY\n", 15U);
        host_link_process(&hl);
        check(strstr(g_sensor, "$VNWRG,75,") != NULL,
              "host_link: switching to BINARY writes the preserved 200 Hz to reg 75 (NO clamp)");

        /* FREQ must NOT be clamped in BINARY mode - the full 1..200 range is valid. */
        g_sensor[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN FREQ 200\n", 12U);
        host_link_process(&hl);
        check(strstr(g_sensor, "$VNWRG,7,50") == NULL,
              "host_link: in BINARY, FREQ 200 is NOT caught by the ASCII clamp");

        host_link_feed(&hl, (const uint8_t *)"VN MODE ASCII\n", 14U);
        host_link_process(&hl);

        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN MODE XYZ\n", 12U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR mode") != NULL,
              "host_link: unknown MODE argument -> VNERR (NOT silently treated as ASCII)");
        check(hl.out_fmt == VN100_FMT_ASCII, "host_link: unknown MODE keeps the current mode");

        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN RAW $VNWRG,5,921600*61\n", 26U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR baud-disabled") != NULL,
              "host_link: a Reg 5 (baud) write via RAW is also REJECTED");

        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN RAW $VNWRG,005,921600\n", 25U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR baud-disabled") != NULL,
              "host_link: zero-padded Reg 5 ('005') via RAW is ALSO REJECTED");

        /* REGRESSION: whitespace/TAB before '$' could bypass the guard
           (strtok(NULL,"") doesn't skip leading delimiters), letting the
           baud write reach the sensor while still returning VNACK. */
        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN RAW  $VNWRG,5,921600*61\n", 27U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR baud-disabled") != NULL,
              "host_link: a DOUBLE SPACE after RAW does NOT bypass the baud guard");

        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN RAW \t$VNWRG,05,921600\n", 25U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR baud-disabled") != NULL,
              "host_link: a TAB after RAW also does NOT bypass the baud guard");

        /* Trimming must NOT break a HARMLESS command: a spaced RAW is still
           forwarded to the sensor (VNACK). */
        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN RAW  $VNRRG,1*71\n", 20U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNACK") != NULL,
              "host_link: a spaced HARMLESS RAW is still forwarded (trimming doesn't break it)");

        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN TYPE 7\n", 10U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR type") != NULL,
              "host_link: 'VN TYPE 7' invalid -> VNERR (only 0|14 - F12)");

        /* strtol (not atoi) checks FULL CONSUMPTION: trailing garbage like
           'FREQ 10abc'/'TYPE 14x' must be explicitly rejected, not silently
           accepted as an unintended value. */
        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN FREQ 10abc\n", 14U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR freq") != NULL,
              "host_link: 'VN FREQ 10abc' trailing garbage REJECTED (strtol)");
        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN TYPE 14x\n", 12U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR type") != NULL,
              "host_link: 'VN TYPE 14x' trailing garbage REJECTED (strtol)");

        /* FIFO overflow is VISIBLE (symmetric with line overflow). */
        g_reply[0] = '\0';
        hl.fifo_drop++;                       /* simulate a byte dropped in the ISR */
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR fifo-overflow") != NULL,
              "host_link: FIFO overflow raises a VISIBLE error");
        g_reply[0] = '\0';
        host_link_process(&hl);               /* must NOT be reported again for the same count */
        check(strstr(g_reply, "fifo-overflow") == NULL,
              "host_link: FIFO overflow is not reported repeatedly (no spam)");

        /* ADOF enum gate (ICD Sec. 3.2.4 Table 3.9) - an in-range but
           non-enum value is REJECTED. Counterpart: the ADOF_VALID anchor
           in tests/test_spec_constants.py. */
        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN FREQ 30\n", 11U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR freq-adof") != NULL,
              "host_link: 'VN FREQ 30' not in enum -> VNERR (ICD Table 3.9)");
        g_reply[0] = '\0';
        host_link_feed(&hl, (const uint8_t *)"VN FREQ 25\n", 11U);
        host_link_process(&hl);
        check(strstr(g_reply, "VNERR") == NULL,
              "host_link: 'VN FREQ 25' is an enum member -> ACCEPTED");

        /* GENERIC error responses - the simulator's parity is measured
           against these. Counterpart: tests/test_simulator.py::test_sim_genel_hata_cevaplari_firmware_ile_ayni
           (this branch must also exist in the simulator, or malformed
           commands are silently swallowed there). */
        {
            struct { const char *line; const char *expect; } cases[] = {
                { "FOO BAR\n",  "VNERR bad"     },   /* tag != "VN"                  */
                { "VN\n",       "VNERR nocmd"   },   /* no command                   */
                { "VN XYZZY\n", "VNERR unknown" },   /* unrecognized command         */
                { "VN TYPE\n",  "VNERR unknown" },   /* no argument -> `arg != NULL` gate */
                { "VN MODE\n",  "VNERR unknown" },
                { "VN FREQ\n",  "VNERR unknown" },
            };
            unsigned k;
            for (k = 0U; k < (sizeof(cases) / sizeof(cases[0])); k++)
            {
                g_reply[0] = '\0';
                host_link_feed(&hl, (const uint8_t *)cases[k].line,
                               (uint16_t)strlen(cases[k].line));
                host_link_process(&hl);
                check(strstr(g_reply, cases[k].expect) != NULL,
                      "host_link: generic error response (sim parity anchor)");
            }
        }
    }

    printf("%s\n", g_fail ? "RESULT: FAIL" : "RESULT: ALL TESTS PASSED");
    return g_fail;
}
