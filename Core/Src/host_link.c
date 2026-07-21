/**
  ******************************************************************************
  * @file    host_link.c
  * @brief   PC <-> STM32 host command channel implementation (HAL-free)
  ******************************************************************************
  */
#include "host_link.h"

#include <ctype.h>
#include <string.h>
#include <stdlib.h>

/* Valid ADOF (Reg 7) values - ICD section 3.2.4 Table 3.9: a CLOSED enum,
   NOT a free range. Mirrors pyvn100/registers.py ADOF_VALID (kept in sync
   by tests/test_spec_constants.py). 0 = stream off.
   Applies only to the ASCII (Reg 7) path - BINARY uses Reg 75 RateDivisor
   (output = 800/divisor) with no such restriction, so the dashboard's
   BINARY_HZ list includes valid-but-non-enum values like 80 Hz. */
static bool adof_is_valid(uint16_t hz)
{
    static const uint16_t valid_values[] = {0U, 1U, 2U, 4U, 5U, 10U, 20U, 25U, 40U, 50U, 100U, 200U};
    size_t i;
    for (i = 0U; i < (sizeof(valid_values) / sizeof(valid_values[0])); i++)
    {
        if (valid_values[i] == hz)
        {
            return true;
        }
    }
    return false;
}

static void reply_str(host_link_t *hl, const char *s)
{
    if (hl->reply != NULL)
    {
        hl->reply(hl->reply_ctx, (const uint8_t *)s, (uint16_t)strlen(s));
    }
}

void host_link_init(host_link_t *hl, vn100_t *vn, host_reply_fn reply, void *reply_ctx)
{
    memset(hl, 0, sizeof(*hl));
    hl->vn = vn;
    hl->reply = reply;
    hl->reply_ctx = reply_ctx;
}

void host_link_feed(host_link_t *hl, const uint8_t *data, uint16_t len)
{
    /* ISR context: push bytes into the FIFO; line assembly happens in main.
       SPSC ring - only head touched here -> safe without locking. Bytes
       drop if it fills (main falling behind a 512-byte FIFO; doesn't
       happen in practice). */
    uint16_t i;
    for (i = 0U; i < len; i++)
    {
        uint16_t next = (uint16_t)((hl->fifo_head + 1U) % HOST_LINK_FIFO_SIZE);
        if (next != hl->fifo_tail)
        {
            hl->fifo[hl->fifo_head] = data[i];
            hl->fifo_head = next;
        }
        else
        {
            /* FIFO full -> byte DROPPED. Line overflow (host_link_process)
               already reports visibly via 'VNERR overflow', so FIFO overflow
               must too. Can't reply from ISR (may block) - just count here;
               main loop gives the visible report (see process). */
            hl->fifo_drop++;
        }
    }
}

static void dispatch(host_link_t *hl, char *line)
{
    /* strtok is fine here: dispatch is only called from the main loop
       (host_link_process), never re-entered. */
    char *tag = strtok(line, " ");
    char *cmd;
    char *arg;
    vn100_status_t st = VN100_OK;

    if ((tag == NULL) || (strcmp(tag, "VN") != 0))
    {
        reply_str(hl, "VNERR bad\r\n");
        return;
    }
    cmd = strtok(NULL, " ");
    if (cmd == NULL)
    {
        reply_str(hl, "VNERR nocmd\r\n");
        return;
    }
    arg = strtok(NULL, "");                  /* everything remaining is a single argument */

    /* Trim leading space/TAB once: strtok(NULL, "") uses an EMPTY delimiter
       set, so it won't skip them itself. Untrimmed, 'VN RAW  $VNWRG,5,...'
       (double space/TAB after RAW) would leave a leading space that slips
       past the '$VNWRG,' prefix check below, letting a baud write reach the
       sensor and break the link. Trimming here also fixes the same
       'VN FREQ  50' / 'VN TYPE  14' / 'VN MODE  ASCII' inconsistency, and the
       trimmed pointer is what's actually sent to the sensor. */
    if (arg != NULL)
    {
        while ((*arg != '\0') && (isspace((unsigned char)*arg) != 0))
        {
            arg++;
        }
    }

    if (strcmp(cmd, "PING") == 0)
    {
        reply_str(hl, "VNPONG\r\n");
        return;
    }
    else if ((strcmp(cmd, "FREQ") == 0) && (arg != NULL))
    {
        /* Rate command: reg 7 ADOF in ASCII mode, reg 75 RateDivisor in
           BINARY mode (docs/protocol.md sec 7) - needed since otherwise the
           binary rate could never change.
           strtol + FULL CONSUMPTION, not atoi: atoi would swallow trailing
           garbage ('FREQ 10abc' -> 10), silently applying a rate the user
           never asked for; it also defaults to 0 for non-numeric input,
           rounds 'FREQ 300' up to 400 Hz via divisor 2 (HIGHER than
           requested), and wraps negatives to 800 Hz via uint16 - all of
           which saturate the link above the 115200 ceiling (binary ~270 Hz).
           Any non-fully-numeric argument is rejected and the range is
           clamped to 1..200 (matching the dashboard's lists and the VCP
           ceiling). */
        char *fson = NULL;
        long hz_raw = strtol(arg, &fson, 10);
        if ((fson == arg) || ((*fson != '\0') && (*fson != '\r') && (*fson != '\n')))
        {
            reply_str(hl, "VNERR freq-range(1..200)\r\n");
            return;
        }
        if ((hz_raw < 1) || (hz_raw > 200))
        {
            reply_str(hl, "VNERR freq-range(1..200)\r\n");
            return;
        }
        uint16_t hz = (uint16_t)hz_raw;
        hl->last_hz = hz;                        /* preserved across MODE switches - the RAW value */
        if (hl->out_fmt == VN100_FMT_BINARY)
        {
            st = vn100_set_output_mode(hl->vn, VN100_FMT_BINARY, hz);
        }
        else
        {
            /* ASCII rate CLAMPED to 50 Hz - same rule as the MODE branch
               below. Clamp applies at USE time only; last_hz keeps the RAW
               value, so 'FREQ 200' then 'MODE BINARY' still gets 200.
               Without the clamp, ASCII would write ADOF=200: a measured
               ASCII frame is 101-118 B, so at 115200 the ceiling is
               ~98-114 Hz - roughly HALF the requested rate - and the relay
               would truncate/drop frames. */
            if (hz > 50U)
            {
                hz = 50U;
            }
            /* Enum check runs AFTER the clamp: clamping to 50 (already a
               valid ADOF) never trips it. What it catches is in-range
               non-enum values like 'VN FREQ 30' - the real sensor returns
               $VNERR for these, and the simulator matches. */
            if (!adof_is_valid(hz))
            {
                reply_str(hl, "VNERR freq-adof(ICD Table 3.9)\r\n");
                return;
            }
            st = vn100_set_async_freq(hl->vn, hz);
        }
    }
    else if ((strcmp(cmd, "TYPE") == 0) && (arg != NULL))
    {
        /* Only 0 (stream off) or 14 (VNYMR) accepted. strtol + FULL
           CONSUMPTION (same rationale as FREQ): a bare atoi would let
           'VN TYPE 14x' parse as 14, or a typo silently kill the stream via
           an ADOR write while still returning VNACK - both are rejected
           here. */
        char *tson = NULL;
        long t = strtol(arg, &tson, 10);
        if ((tson == arg) || ((*tson != '\0') && (*tson != '\r') && (*tson != '\n'))
            || ((t != 0) && (t != 14)))
        {
            reply_str(hl, "VNERR type(0|14)\r\n");
            return;
        }
        st = vn100_set_async_type(hl->vn, (uint16_t)t);
    }
    else if (strcmp(cmd, "BAUD") == 0)
    {
        /* VN BAUD disabled: writing a new baud switches the sensor
           immediately, but USART6 stays at 115200 -> the link BREAKS and
           needs a reset. Safely synchronizing both sides (write sensor,
           also reconfigure USART6) needs hardware testing, so the command
           is rejected for now to avoid losing the link during bring-up. */
        reply_str(hl, "VNERR baud-disabled\r\n");
        return;
    }
    else if (strcmp(cmd, "TARE") == 0)
    {
        /* $VNTAR is NOT in this firmware's command list (ICD section 1.3:
           VNRRG/VNWRG/VNWNV/VNRFS/VNRST/VNFWU/VNKMD/VNKAD/VNASY/VNSGB/VNBOM)
           - the field sensor returns $VNERR,04 (Invalid Command) for it.
           Still FORWARDED anyway to keep the bridge transparent: the
           sensor's real response (echo or $VNERR) reaches the PC console,
           and the dashboard's Tare button enables/disables based on
           firmware capability. FW v2.x sensors still support it, so it's
           not rejected here. */
        st = vn100_tare(hl->vn);
    }
    else if (strcmp(cmd, "SAVE") == 0)
    {
        st = vn100_write_settings(hl->vn);
    }
    else if (strcmp(cmd, "FACTORY") == 0)
    {
        st = vn100_restore_factory(hl->vn);
    }
    else if ((strcmp(cmd, "MODE") == 0) && (arg != NULL))
    {
        /* Output mode: ASCII for demos, BINARY for normal operation
           (docs/protocol.md sec 4.3). Sets both the SENSOR format (reg 6 /
           reg 75) and the RELAY format (out_fmt), so the STM32->PC stream
           actually matches what was selected, not just what's displayed.
           Rate: PRESERVES the last VN FREQ if one was set, else the mode's
           default (binary 200, ascii 50) - so 'FREQ 10' then 'MODE ASCII'
           doesn't get overwritten. Preserved rate is CLAMPED to 50 Hz in
           ASCII: binary 200 carried into ASCII would write ADOF=200,
           doubling the VCP ceiling (~90 Hz ASCII) and causing the relay to
           truncate/drop frames (the dashboard list caps ASCII at 50 too).
           Only 'ASCII'/'BINARY' accepted - an unknown argument is rejected,
           not silently treated as ASCII.
           If the UART write fails, out_fmt stays UNCHANGED and VNERR is
           returned (same guard as the FREQ path) - otherwise a USART6
           fault would make MODE lie and the relay format would diverge
           from the sensor's actual mode. Replies with a dedicated ack
           (VNMODE ...). */
        if (strcmp(arg, "BINARY") == 0)
        {
            st = vn100_set_output_mode(hl->vn, VN100_FMT_BINARY,
                                       (hl->last_hz != 0U) ? hl->last_hz : 200U);
            if (st == VN100_OK)
            {
                hl->out_fmt = VN100_FMT_BINARY;
                reply_str(hl, "VNMODE BINARY\r\n");
            }
            else
            {
                reply_str(hl, "VNERR fail\r\n");
            }
        }
        else if (strcmp(arg, "ASCII") == 0)
        {
            uint16_t hz = (hl->last_hz != 0U) ? hl->last_hz : 50U;
            if (hz > 50U)
            {
                hz = 50U;
            }
            st = vn100_set_output_mode(hl->vn, VN100_FMT_ASCII, hz);
            if (st == VN100_OK)
            {
                hl->out_fmt = VN100_FMT_ASCII;
                reply_str(hl, "VNMODE ASCII\r\n");
            }
            else
            {
                reply_str(hl, "VNERR fail\r\n");
            }
        }
        else
        {
            reply_str(hl, "VNERR mode(ASCII|BINARY)\r\n");
        }
        return;   /* dedicated ack already sent - skip the generic VNACK */
    }
    else if ((strcmp(cmd, "RAW") == 0) && (arg != NULL))
    {
        /* 'VN BAUD' is rejected, but 'VN RAW $VNWRG,5,...' could bypass that
           and cause the same one-sided baud write (BREAKING the link until
           reset) - so reg 5 writes are blocked on the RAW path too. A plain
           literal-prefix check isn't enough, since the reg field is parsed
           as an integer and '005' would slip through - so the reg number is
           parsed with atoi and rejected if it equals 5 (covers 5/05/005/
           leading whitespace). */
        if (strncmp(arg, "$VNWRG,", 7) == 0)
        {
            const char *p = arg + 7;
            while (*p == ' ')
            {
                p++;
            }
            if (atoi(p) == 5)
            {
                reply_str(hl, "VNERR baud-disabled\r\n");
                return;
            }
        }
        st = vn100_send_raw(hl->vn, arg);   /* return VNERR on failure */
        if (st == VN100_OK)
        {
            /* Terminator write's return value is checked too: if the body
               sent but the terminator DID NOT (TX timeout/link drop), the
               sensor command never completes as a line, yet the PC would
               still get VNACK - a half command plus a false ack. */
            st = vn100_send_raw(hl->vn, "\r\n");
        }
    }
    else
    {
        reply_str(hl, "VNERR unknown\r\n");
        return;
    }

    /* VNACK only means the command was WRITTEN to the sensor, not that the
       sensor ACCEPTED it - send_buf only checks the UART write succeeded,
       it never waits for a reply. The actual accept/reject ($VNWRG echo /
       $VNERR) reaches the PC via a separate path (main.c
       vn_forward_response); during bring-up watch that, not the ack. */
    reply_str(hl, (st == VN100_OK) ? "VNACK\r\n" : "VNERR fail\r\n");
}

void host_link_process(host_link_t *hl)
{
    /* Report ISR byte drops ONCE, visibly (symmetric with the line-overflow
       report); not re-reported while the counter is unchanged, so no
       console spam. This is the only observable answer to "why did my
       command never run?". */
    if (hl->fifo_drop != hl->fifo_drop_reported)
    {
        hl->fifo_drop_reported = hl->fifo_drop;
        reply_str(hl, "VNERR fifo-overflow\r\n");
    }

    /* Main loop: drain the FIFO, assemble complete line(s), process EACH.
       FIFO+loop instead of a single slot -> multiple commands arriving in
       one pass are NOT dropped. */
    while (hl->fifo_tail != hl->fifo_head)
    {
        char c = (char)hl->fifo[hl->fifo_tail];
        hl->fifo_tail = (uint16_t)((hl->fifo_tail + 1U) % HOST_LINK_FIFO_SIZE);

        if (c == '\r')
        {
            continue;
        }
        if (c == '\n')
        {
            if (hl->overflow)
            {
                hl->overflow = false;
                hl->pos = 0U;
                reply_str(hl, "VNERR overflow\r\n");   /* visible error instead of a silent loss */
            }
            else
            {
                hl->line[hl->pos] = '\0';
                if (hl->pos > 0U)
                {
                    dispatch(hl, hl->line);            /* complete line -> parse + apply */
                }
                hl->pos = 0U;
            }
        }
        else if (hl->pos < (HOST_LINK_LINE_MAX - 1U))
        {
            hl->line[hl->pos++] = c;
        }
        else
        {
            hl->overflow = true;                       /* line too long -> overflow (see '\n') */
        }
    }
}
