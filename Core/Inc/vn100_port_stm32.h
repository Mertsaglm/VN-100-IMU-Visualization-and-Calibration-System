/**
  ******************************************************************************
  * @file    vn100_port_stm32.h
  * @brief   VN-100 STM32 HAL port (LOW-level, platform-SPECIFIC)
  ******************************************************************************
  * Connects the portable core to the STM32 HAL: feeds bytes received over
  * UART + DMA (circular) + IDLE-line into vn100_rx_feed(); provides the
  * write/millis/critical-section functions. Porting to a different board
  * means changing ONLY this file.
  ******************************************************************************
  */
#ifndef VN100_PORT_STM32_H
#define VN100_PORT_STM32_H

#include "stm32f7xx_hal.h"
#include "vn100.h"
#include "vn100_port.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VN100_STM32_DMA_BUF  512U   /**< DMA circular buffer size (bytes) */

/** STM32 port context. */
typedef struct
{
    UART_HandleTypeDef *huart;                 /**< UART connected to the VN-100 (USART6) */
    vn100_port_t        port;                  /**< port interface handed to the core */
    vn100_t            *vn;                     /**< core to feed */
    uint8_t             dma_buf[VN100_STM32_DMA_BUF];
    uint16_t            last_pos;              /**< last DMA position read */
} vn100_stm32_t;

/** Initializes the port and the core (DMA is not started yet). */
vn100_status_t vn100_stm32_init(vn100_stm32_t *s, vn100_t *vn, UART_HandleTypeDef *huart,
                                vn100_packet_cb on_packet, vn100_error_cb on_error, void *user);

/** Starts circular DMA RX + the IDLE interrupt. */
vn100_status_t vn100_stm32_start(vn100_stm32_t *s);

/** Stops DMA + IDLE. */
vn100_status_t vn100_stm32_stop(vn100_stm32_t *s);

/** Called from USART6_IRQHandler: handles the IDLE flag and feeds in new bytes. */
void vn100_stm32_on_uart_idle(vn100_stm32_t *s);

/** Feeds any new bytes in the DMA ring into the core (independent of the
 *  IDLE flag). Called from the IDLE ISR and from DMA half/full-transfer
 *  callbacks (FW-U2). */
void vn100_stm32_drain(vn100_stm32_t *s);

#ifdef __cplusplus
}
#endif

#endif /* VN100_PORT_STM32_H */
