/**
  ******************************************************************************
  * @file    host_link.h
  * @brief   PC <-> STM32 host command channel (docs/protocol.md, section 6)
  ******************************************************************************
  * Parses "VN <CMD> [arg]" lines from the PC (dashboard) and applies them to
  * the VN-100. Push model: bytes are fed in via host_link_feed() (from the
  * ISR); once a full line has accumulated, host_link_process() (called from
  * the main loop) parses, applies, and replies. This keeps the ISR free of
  * blocking transmit calls.
  ******************************************************************************
  */
#ifndef HOST_LINK_H
#define HOST_LINK_H

#include <stdint.h>
#include <stdbool.h>
#include "vn100.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HOST_LINK_FIFO_SIZE 512  /* ISR->main command byte FIFO (absorbs a burst of commands) */
#define HOST_LINK_LINE_MAX  192  /* one command line: a 12-float Reg 23 write needs ~150 B (96 is too tight) */

/** Reply-writer function (transmits over USART3 on the STM32). */
typedef int (*host_reply_fn)(void *ctx, const uint8_t *data, uint16_t len);

typedef struct
{
    vn100_t      *vn;
    host_reply_fn reply;
    void         *reply_ctx;

    /* ISR->main byte FIFO (SPSC ring): ISR writes head, main reads tail. Lets
       back-to-back commands queue up instead of being dropped while a line is
       still pending. */
    volatile uint8_t  fifo[HOST_LINK_FIFO_SIZE];
    volatile uint16_t fifo_head;
    volatile uint16_t fifo_tail;
    /* Bytes dropped because the FIFO was full. A line overflow already yields a
       visible 'VNERR overflow', so this must not stay silent either (symmetric
       failure modes). Tracked here since the ISR can't reply; main loop reports
       it once, visibly. */
    volatile uint32_t fifo_drop;
    uint32_t          fifo_drop_reported;   /* last reported value (avoids repeat spam) */

    /* Line assembly (used only by host_link_process / main) */
    char          line[HOST_LINK_LINE_MAX];
    uint16_t      pos;
    bool          overflow;   /* line overflow -> swallow until next '\n', then VNERR (no silent loss) */

    /* Telemetry format sent to the PC (set via VN MODE): ASCII for demos,
       BINARY for normal operation. main.c's vn_relay() reads this to pick
       the stream format. */
    vn100_fmt_t   out_fmt;
    uint16_t      last_hz;    /* last output Hz set via VN FREQ; 0 = unset -> MODE default */
} host_link_t;

void host_link_init(host_link_t *hl, vn100_t *vn, host_reply_fn reply, void *reply_ctx);

/** Called from the ISR: accumulates incoming bytes into a line. */
void host_link_feed(host_link_t *hl, const uint8_t *data, uint16_t len);

/** Called from the main loop: parses, applies, and replies to a ready command. */
void host_link_process(host_link_t *hl);

#ifdef __cplusplus
}
#endif

#endif /* HOST_LINK_H */
