/**
  ******************************************************************************
  * @file    vn100.h
  * @brief   VN-100 high-level API (HIGH level, platform-INDEPENDENT)
  ******************************************************************************
  * C mirror of the pyvn100.vn100.VN100 class. Operates through a vn100_port_t;
  * incoming bytes are "pushed" in via vn100_rx_feed() (from the IDLE ISR on
  * STM32, from a read loop on PC). There is NO LED/debug handling in the
  * core — the application (main.c) handles that via callbacks, keeping the
  * core fully portable.
  ******************************************************************************
  */
#ifndef VN100_H
#define VN100_H

#include <stdint.h>
#include <stdbool.h>
#include "vn100_types.h"
#include "vn100_port.h"
#include "vn100_binary.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VN100_MSG_MAX   256U  /**< Single-message assembly buffer (12-float Reg 47 reply ~140 B + margin) */
#define VN100_RESP_MAX  256U  /**< Command response ($VNRRG/$VNWRG) slot size (Reg 23/47 must not truncate) */
#define VN100_RESP_SLOTS 4U   /**< Response queue depth (SPSC sentinel ring -> 3 usable slots;
                                   a snapshot needs Reg 23+44=2, so it fits). Keeps back-to-back
                                   responses from overwriting each other. */

/** Output format — for vn100_set_output_mode(). */
typedef enum { VN100_FMT_ASCII = 0, VN100_FMT_BINARY = 1 } vn100_fmt_t;

/** Dual-mode RX state machine state (auto-detects ASCII '$' vs. binary 0xFA). */
typedef enum { VN100_RX_IDLE = 0, VN100_RX_ASCII, VN100_RX_BIN } vn100_rx_mode_t;

/** Successful-packet callback (optional). */
typedef void (*vn100_packet_cb)(const vn100_data_t *d, void *user);
/** Error callback (optional). */
typedef void (*vn100_error_cb)(vn100_status_t err, void *user);

/** Driver context — user code must not touch the fields directly. */
typedef struct
{
    const vn100_port_t *port;
    vn100_packet_cb on_packet;
    vn100_error_cb  on_error;
    void           *user;

    /* Dual-mode RX state machine (ASCII line + binary frame) */
    char            msg[VN100_MSG_MAX];
    uint16_t        msg_pos;
    vn100_rx_mode_t rx_mode;
    uint8_t         bin[VN100_BIN_FRAME_LEN];
    uint16_t        bin_pos;

    /* Latest data + statistics */
    vn100_data_t   data;
    volatile bool  has_data;
    /* Written from the ISR (commit_data), read from the main loop (vn_relay) -> volatile */
    volatile uint32_t packet_count;
    volatile uint32_t error_count;
    volatile uint32_t byte_count;

    /* Command response queue: $VN... lines from the sensor (except VNYMR),
       held for the host bridge (main.c) to forward to the PC verbatim. SPSC
       ring (see VN100_RESP_SLOTS) avoids overwriting on back-to-back replies.
       store_response (ISR) writes head, vn100_take_response (main) reads tail. */
    char             resp_q[VN100_RESP_SLOTS][VN100_RESP_MAX];
    uint16_t         resp_len[VN100_RESP_SLOTS];
    volatile uint8_t resp_head;
    volatile uint8_t resp_tail;
} vn100_t;

/** Initializes the core. */
vn100_status_t vn100_init(vn100_t *v, const vn100_port_t *port,
                          vn100_packet_cb on_packet, vn100_error_cb on_error, void *user);

/** Feeds incoming raw bytes into the core (push model). */
void vn100_rx_feed(vn100_t *v, const uint8_t *data, uint16_t len);

/** Copies the latest valid measurement. Returns false if none is available. */
bool vn100_get_data(vn100_t *v, vn100_data_t *out);

/** Copies the latest measurement AND its packet_count in a SINGLE critical section (for the relay race). */
bool vn100_get_data_counted(vn100_t *v, vn100_data_t *out, uint32_t *count);

/**
 * If a command response ($VNRRG/$VNWRG/$VNERR) is pending, copies it to
 * out and clears the mailbox. Called from the main loop; the host bridge
 * forwards it to the PC. Returns false if there is no response. (This is
 * how register reads such as Reg 46/47 reach the PC — the sensor's reply
 * arrives on USART6 and the core captures it.)
 */
bool vn100_take_response(vn100_t *v, char *out, uint16_t outsize, uint16_t *out_len);

/* -- Commands (via port->write) --
   NOTE: read_register / set_baudrate / configure / write_register_f /
   write_mag_calibration / set_gyro_bias are intentionally ABSENT from this
   API (rationale in vn100.c). Unverified register writes from firmware would
   be a second path around the project's "VNACK != sensor accepted" rule;
   writes are verified by reading them back on the PC side instead. */
vn100_status_t vn100_set_async_type(vn100_t *v, uint16_t ador);
vn100_status_t vn100_set_async_freq(vn100_t *v, uint16_t hz);
vn100_status_t vn100_tare(vn100_t *v);
vn100_status_t vn100_write_settings(vn100_t *v);
vn100_status_t vn100_restore_factory(vn100_t *v);

/** Sends raw text directly to the sensor (for the host RAW command). */
vn100_status_t vn100_send_raw(vn100_t *v, const char *text);

/** Configures the Binary Output register (75): async_mode + rate_divisor (output Hz = 800/rate_divisor). */
vn100_status_t vn100_set_binary_output(vn100_t *v, uint16_t async_mode, uint16_t rate_divisor);

/**
 * Selects the output mode (docs/protocol.md, section 4.3): ASCII for demos,
 * binary for normal operation. ASCII -> reg6=VNYMR + reg75 off; BINARY ->
 * reg6=off + reg75 (rate_hz). The registers are independent so no reflash is
 * needed; the RX parser auto-detects either.
 */
vn100_status_t vn100_set_output_mode(vn100_t *v, vn100_fmt_t fmt, uint16_t rate_hz);

#ifdef __cplusplus
}
#endif

#endif /* VN100_H */
