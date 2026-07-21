/**
  ******************************************************************************
  * @file    vn100.c
  * @brief   VN-100 high-level core implementation (HAL-free, portable)
  ******************************************************************************
  */
#include "vn100.h"
#include "vn100_protocol.h"
#include "vn100_registers.h"

#include <string.h>

/* -- Helpers -------------------------------------------------------- */

static void crit_enter(vn100_t *v)
{
    if (v->port->enter_critical != NULL)
    {
        v->port->enter_critical(v->port->ctx);
    }
}

static void crit_exit(vn100_t *v)
{
    if (v->port->exit_critical != NULL)
    {
        v->port->exit_critical(v->port->ctx);
    }
}

static uint32_t now_ms(vn100_t *v)
{
    return (v->port->millis != NULL) ? v->port->millis(v->port->ctx) : 0U;
}

static vn100_status_t send_buf(vn100_t *v, const char *buf, int n)
{
    if (n <= 0)
    {
        return VN100_ERR;
    }
    if (v->port->write == NULL)
    {
        return VN100_ERR;
    }
    return (v->port->write(v->port->ctx, (const uint8_t *)buf, (uint16_t)n) >= 0)
               ? VN100_OK : VN100_ERR;
}

/* -- Initialization --------------------------------------------------- */

vn100_status_t vn100_init(vn100_t *v, const vn100_port_t *port,
                          vn100_packet_cb on_packet, vn100_error_cb on_error, void *user)
{
    if ((v == NULL) || (port == NULL))
    {
        return VN100_ERR_PARAM;
    }
    memset(v, 0, sizeof(*v));
    v->port      = port;
    v->on_packet = on_packet;
    v->on_error  = on_error;
    v->user      = user;
    return VN100_OK;
}

/* -- RX / state machine (push) ------------------------------------- */

static void store_response(vn100_t *v)
{
    /* Append a command response from the sensor ($VN..., except VNYMR) to
       the response QUEUE. SPSC ring (ISR writes, main reads), so back-to-
       back responses (e.g. Reg 23+44) don't overwrite each other. If full
       (main fell behind; in practice the 4 slots never fill), the NEWEST
       one is dropped - only head advances, tail untouched (safe without
       locking). */
    uint16_t n = v->msg_pos;
    uint8_t next = (uint8_t)((v->resp_head + 1U) % VN100_RESP_SLOTS);
    if (next == v->resp_tail)
    {
        return;   /* queue full -> drop the new response (leave tail alone) */
    }
    if (n >= VN100_RESP_MAX)
    {
        n = VN100_RESP_MAX - 1U;
    }
    memcpy(v->resp_q[v->resp_head], v->msg, n);
    v->resp_q[v->resp_head][n] = '\0';
    v->resp_len[v->resp_head]  = n;
    v->resp_head = next;
}

/* Commits a parsed measurement (ASCII or binary) + updates stats/callback.
   Both the ASCII and binary paths share this helper (single source of truth). */
static void commit_data(vn100_t *v, vn100_data_t *d)
{
    d->timestamp = now_ms(v);
    crit_enter(v);
    v->data = *d;
    v->has_data = true;
    v->packet_count++;
    crit_exit(v);
    if (v->on_packet != NULL)
    {
        /* Callback receives the caller's local `d`, not the SHARED v->data:
           reading v->data OUTSIDE the critical section let the next ISR
           frame overwrite it mid-callback (torn read). `d` isn't exposed
           elsewhere during the commit and matches v->data exactly. */
        v->on_packet(d, v->user);
    }
}

static void handle_complete_msg(vn100_t *v)
{
    vn100_data_t d;

    if ((v->msg_pos >= 6U) && (memcmp(v->msg, "$VNYMR", 6) == 0))
    {
        /* Asynchronous telemetry ($VNYMR) */
        vn100_status_t st = vn100_parse_vnymr(v->msg, v->msg_pos, &d);
        if (st == VN100_OK)
        {
            commit_data(v, &d);
        }
        else
        {
            v->error_count++;
            if (v->on_error != NULL)
            {
                v->on_error(st, v->user);
            }
        }
    }
    else if ((v->msg_pos >= 3U) && (memcmp(v->msg, "$VN", 3) == 0))
    {
        /* Command response ($VNRRG/$VNWRG/$VNERR) -> let the host bridge forward it to the PC */
        store_response(v);
    }
}

/* Decodes and commits a 42-byte binary frame (once RX_BIN completes). */
/**
 * @brief  Recovers an ASCII response SWALLOWED by a failed binary frame.
 * @return true if a '$' was found inside and ASCII accumulation is now
 *              active (caller must NOT touch rx_mode); false if nothing to
 *              recover.
 *
 * WHY: in BINARY mode, losing one byte SHIFTS the frame - the parser still
 * unconditionally consumes 42 bytes, so the FOLLOWING command response
 * ($VNRRG/$VNERR) ends up inside that window. If the CRC fails and all 42
 * bytes are discarded, the response is lost SILENTLY - but "verify by
 * reading back" (docs/protocol.md sec 8.2) depends on exactly that
 * response as the ONLY proof a calibration write was accepted.
 * Since the frame is garbage anyway, continuing as ASCII from the '$'
 * costs nothing and may recover it. No recursion: only rx_mode switches to
 * ASCII, remaining bytes flow through normally.
 */
static bool recover_ascii_from_binary(vn100_t *v)
{
    uint16_t i;
    /* Start at i=1: [0] is the sync byte itself. */
    for (i = 1U; i < v->bin_pos; i++)
    {
        if (v->bin[i] == (uint8_t)'$')
        {
            uint16_t n = (uint16_t)(v->bin_pos - i);
            if (n > VN100_MSG_MAX)
            {
                n = VN100_MSG_MAX;           /* won't overflow; the rest keeps arriving from the stream */
            }
            memcpy(v->msg, &v->bin[i], n);
            v->msg_pos = n;
            v->rx_mode = VN100_RX_ASCII;     /* completes when '\n' arrives */
            return true;
        }
    }
    return false;
}

static void handle_binary_frame(vn100_t *v)
{
    vn100_data_t d;
    vn100_status_t st = vn100_binary_decode(v->bin, v->bin_pos, &d);
    if (st == VN100_OK)
    {
        commit_data(v, &d);
    }
    else
    {
        v->error_count++;
        if (v->on_error != NULL)
        {
            v->on_error(st, v->user);
        }
        /* Frame is corrupt -> check whether a command response is trapped inside, and recover it. */
        (void)recover_ascii_from_binary(v);
    }
}

/* Starts a new frame from one byte: '$'->ASCII, 0xFA->binary, else garbage (IDLE). */
static void start_frame(vn100_t *v, uint8_t b)
{
    if (b == (uint8_t)'$')
    {
        v->msg[0] = '$';
        v->msg_pos = 1U;
        v->rx_mode = VN100_RX_ASCII;
    }
    else if (b == VN100_BIN_SYNC)
    {
        v->bin[0] = b;
        v->bin_pos = 1U;
        v->rx_mode = VN100_RX_BIN;
    }
    /* otherwise: garbage between frames - ignore (stay IDLE) */
}

bool vn100_take_response(vn100_t *v, char *out, uint16_t outsize, uint16_t *out_len)
{
    bool have;
    uint16_t n = 0U;

    if ((v == NULL) || (out == NULL) || (outsize == 0U))
    {
        return false;
    }
    crit_enter(v);
    have = (v->resp_tail != v->resp_head);   /* anything pending in the queue? */
    if (have)
    {
        n = v->resp_len[v->resp_tail];
        if (n > (uint16_t)(outsize - 1U))
        {
            n = (uint16_t)(outsize - 1U);
        }
        memcpy(out, v->resp_q[v->resp_tail], n);
        out[n] = '\0';
        v->resp_tail = (uint8_t)((v->resp_tail + 1U) % VN100_RESP_SLOTS);
    }
    crit_exit(v);
    if (have && (out_len != NULL))
    {
        *out_len = n;
    }
    return have;
}

void vn100_rx_feed(vn100_t *v, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    if ((v == NULL) || (data == NULL))
    {
        return;
    }

    /* Dual-mode auto-detecting state machine: pulls both ASCII lines
       ('$'...'\n') and binary frames (0xFA...42B) from the same stream.
       REQUIRED even in binary mode, since command responses ($VNRRG) still
       arrive as ASCII (docs/protocol.md sec 4.3). */
    for (i = 0U; i < len; i++)
    {
        uint8_t b = data[i];
        v->byte_count++;

        switch (v->rx_mode)
        {
        case VN100_RX_ASCII:
            if (b == (uint8_t)'$')
            {
                v->msg[0] = '$';         /* a dropped '\n' -> resync */
                v->msg_pos = 1U;
            }
            else if (v->msg_pos < VN100_MSG_MAX)
            {
                v->msg[v->msg_pos++] = (char)b;
                if (b == (uint8_t)'\n')
                {
                    handle_complete_msg(v);
                    v->rx_mode = VN100_RX_IDLE;
                }
            }
            else
            {
                v->error_count++;        /* overflow */
                v->rx_mode = VN100_RX_IDLE;
            }
            break;

        case VN100_RX_BIN:
            /* Early header validation (groups=0x01, fields=0x0128 LE): bail
               out immediately on a wrong value and re-evaluate the byte ->
               robust resync. */
            if (((v->bin_pos == 1U) && (b != 0x01U)) ||
                ((v->bin_pos == 2U) && (b != 0x28U)) ||
                ((v->bin_pos == 3U) && (b != 0x01U)))
            {
                v->rx_mode = VN100_RX_IDLE;
                start_frame(v, b);
            }
            else
            {
                v->bin[v->bin_pos++] = b;
                if (v->bin_pos >= VN100_BIN_FRAME_LEN)
                {
                    handle_binary_frame(v);
                    /* If handle_binary_frame finds a '$' inside a corrupt
                       frame, it switches rx_mode to ASCII to recover the
                       swallowed response - do NOT force IDLE here in that
                       case (it would cancel the recovery immediately). */
                    if (v->rx_mode == VN100_RX_BIN)
                    {
                        v->rx_mode = VN100_RX_IDLE;
                    }
                }
            }
            break;

        case VN100_RX_IDLE:
        default:
            start_frame(v, b);
            break;
        }
    }
}

bool vn100_get_data(vn100_t *v, vn100_data_t *out)
{
    if ((v == NULL) || (out == NULL) || !v->has_data)
    {
        return false;
    }
    crit_enter(v);
    *out = v->data;
    crit_exit(v);
    return true;
}

bool vn100_get_data_counted(vn100_t *v, vn100_data_t *out, uint32_t *count)
{
    if ((v == NULL) || (out == NULL) || (count == NULL) || !v->has_data)
    {
        return false;
    }
    crit_enter(v);
    *out   = v->data;
    *count = v->packet_count;
    crit_exit(v);
    return true;
}

/* -- Commands --------------------------------------------------------- */

vn100_status_t vn100_set_async_type(vn100_t *v, uint16_t ador)
{
    char b[40];
    int len = vn100_cmd_write_register_u32(b, sizeof(b), VN100_REG_ASYNC_OUT_TYPE, ador);
    return send_buf(v, b, len);
}

vn100_status_t vn100_set_async_freq(vn100_t *v, uint16_t hz)
{
    char b[40];
    int len = vn100_cmd_write_register_u32(b, sizeof(b), VN100_REG_ASYNC_OUT_FREQ, hz);
    return send_buf(v, b, len);
}

/* NOTE: no vn100_set_baudrate() (Reg 5) in this API - baud changes are
   deliberately locked out: host_link.c rejects 'VN BAUD' and closes the
   'VN RAW $VNWRG,5/05/005' escape (incl. leading space/TAB), and the
   simulator rejects it too, because a one-sided baud change BREAKS the
   sensor<->STM32 link. This function would be a third path around those
   guards. The Python side has no equivalent, for the same reason
   (vn100.py). */

vn100_status_t vn100_tare(vn100_t *v)
{
    char b[24];
    int len = vn100_cmd_simple(b, sizeof(b), "VNTAR");
    return send_buf(v, b, len);
}

vn100_status_t vn100_write_settings(vn100_t *v)
{
    char b[24];
    int len = vn100_cmd_simple(b, sizeof(b), "VNWNV");
    return send_buf(v, b, len);
}

vn100_status_t vn100_restore_factory(vn100_t *v)
{
    char b[24];
    int len = vn100_cmd_simple(b, sizeof(b), "VNRFS");
    return send_buf(v, b, len);
}

vn100_status_t vn100_send_raw(vn100_t *v, const char *text)
{
    if ((v == NULL) || (text == NULL))
    {
        return VN100_ERR_PARAM;
    }
    return send_buf(v, text, (int)strlen(text));
}

/* NOTE: vn100_configure() / vn100_write_register_f() /
   vn100_write_mag_calibration() are not part of this API. Calibration
   writes happen on the PC side and are verified by READING BACK
   (pyvn100.VN100.write_register_verified); the STM32 is only a transparent
   bridge ('VN RAW' passthrough). An unverified 12-float write path from
   firmware would violate the project's "VNACK != sensor accepted" rule via
   a second door. */

vn100_status_t vn100_set_binary_output(vn100_t *v, uint16_t async_mode, uint16_t rate_divisor)
{
    char b[48];
    int len = vn100_cmd_binary_output(b, sizeof(b), VN100_REG_BINARY_OUTPUT_1,
                                       async_mode, rate_divisor,
                                       VN100_BIN_GROUP_COMMON,
                                       VN100_BIN_FIELDS_YPR_RATE_ACCEL);
    return send_buf(v, b, len);
}

vn100_status_t vn100_set_output_mode(vn100_t *v, vn100_fmt_t fmt, uint16_t rate_hz)
{
    vn100_status_t st;
    if (fmt == VN100_FMT_BINARY)
    {
        uint16_t div = (rate_hz > 0U) ? (uint16_t)(VN100_IMU_RATE_HZ / rate_hz) : 4U;
        if (div == 0U)
        {
            div = 1U;
        }
        st = vn100_set_async_type(v, VN100_ADOR_OFF);        /* turn off ASCII */
        if (st != VN100_OK)
        {
            return st;
        }
        return vn100_set_binary_output(v, VN100_SENSOR_ASYNC_PORT, div);
    }
    /* ASCII: turn off binary + turn on VNYMR + set frequency */
    st = vn100_set_binary_output(v, VN100_ASYNC_OFF, 4U);
    if (st != VN100_OK)
    {
        return st;
    }
    st = vn100_set_async_type(v, VN100_ADOR_VNYMR);
    if (st != VN100_OK)
    {
        return st;
    }
    return vn100_set_async_freq(v, rate_hz);
}

/* NOTE: vn100_set_gyro_bias() is not part of this API, nor is there a host
   verb ('VN SGB'). Gyro bias is written from the PC side via
   'VN RAW $VNSGB' (dashboard/gyro_bias_dialog.py), because the sensor must
   be verified ACTUALLY stationary BEFORE the write - a gate only buildable
   on the PC side. */
