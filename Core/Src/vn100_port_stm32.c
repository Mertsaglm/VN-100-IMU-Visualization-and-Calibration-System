/**
  ******************************************************************************
  * @file    vn100_port_stm32.c
  * @brief   VN-100 STM32 HAL port implementation (platform-SPECIFIC)
  ******************************************************************************
  * Operating principle:
  *   1. HAL_UART_Receive_DMA keeps a circular buffer continuously filled
  *   2. When a message ends, the line goes IDLE -> USART6_IRQHandler fires
  *   3. vn100_stm32_on_uart_idle(): reads the DMA head and feeds new bytes
  *      into the portable core via vn100_rx_feed() (block by block)
  ******************************************************************************
  */
#include "vn100_port_stm32.h"

#include <string.h>

/* Single-level critical section (main and ISR never nest: main disables
   IRQs). INVARIANT: this ONE static primask is safe because USART6 (IDLE) +
   DMA2_Stream1 are the only ISR paths entering the core's critical section,
   and both share NVIC priority 1, so neither can preempt the other; USART3
   (priority 5) never enters it (only pushes to the FIFO). Changing these
   priority assignments (main.c/msp) BREAKS this invariant - a primask
   stack would then be required. */
static uint32_t s_primask;

/* -- Port functions --------------------------------------------------- */

static int stm32_write(void *ctx, const uint8_t *data, uint16_t len)
{
    UART_HandleTypeDef *h = (UART_HandleTypeDef *)ctx;
    return (HAL_UART_Transmit(h, (uint8_t *)data, len, 100U) == HAL_OK) ? (int)len : -1;
}

static uint32_t stm32_millis(void *ctx)
{
    (void)ctx;
    return HAL_GetTick();
}

static void stm32_crit_enter(void *ctx)
{
    (void)ctx;
    s_primask = __get_PRIMASK();
    __disable_irq();
}

static void stm32_crit_exit(void *ctx)
{
    (void)ctx;
    __set_PRIMASK(s_primask);
}

/* -- Initialization --------------------------------------------------- */

vn100_status_t vn100_stm32_init(vn100_stm32_t *s, vn100_t *vn, UART_HandleTypeDef *huart,
                                vn100_packet_cb on_packet, vn100_error_cb on_error, void *user)
{
    if ((s == NULL) || (vn == NULL) || (huart == NULL))
    {
        return VN100_ERR_PARAM;
    }

    memset(s, 0, sizeof(*s));
    s->huart    = huart;
    s->vn       = vn;
    s->last_pos = 0U;

    s->port.write          = stm32_write;
    s->port.millis         = stm32_millis;
    s->port.enter_critical = stm32_crit_enter;
    s->port.exit_critical  = stm32_crit_exit;
    s->port.ctx            = huart;

    return vn100_init(vn, &s->port, on_packet, on_error, user);
}

vn100_status_t vn100_stm32_start(vn100_stm32_t *s)
{
    if (s == NULL)
    {
        return VN100_ERR_PARAM;
    }

    s->last_pos = 0U;

    /* ORDER MATTERS: start DMA FIRST, then enable the IDLE interrupt.
       Reversed (the old behavior), there was a narrow window where an IDLE
       interrupt could arrive while IDLE was enabled but DMA wasn't running
       yet, letting drain() read the stopped DMA's STALE NDTR and feed old
       buffer contents into the core. Also, if DMA fails to start, the IDLE
       interrupt is NOT left enabled (otherwise the IDLE ISR would fire
       forever with no DMA behind it). */
    if (HAL_UART_Receive_DMA(s->huart, s->dma_buf, VN100_STM32_DMA_BUF) != HAL_OK)
    {
        __HAL_UART_DISABLE_IT(s->huart, UART_IT_IDLE);
        return VN100_ERR;
    }

    __HAL_UART_CLEAR_IDLEFLAG(s->huart);
    __HAL_UART_ENABLE_IT(s->huart, UART_IT_IDLE);
    return VN100_OK;
}

vn100_status_t vn100_stm32_stop(vn100_stm32_t *s)
{
    if (s == NULL)
    {
        return VN100_ERR_PARAM;
    }
    __HAL_UART_DISABLE_IT(s->huart, UART_IT_IDLE);
    HAL_UART_DMAStop(s->huart);
    return VN100_OK;
}

/* -- IDLE ISR bridge ---------------------------------------------------- */

void vn100_stm32_drain(vn100_stm32_t *s)
{
    uint16_t head;
    uint16_t pos;

    if (s == NULL)
    {
        return;
    }

    /* Position DMA has written up to = buffer size - remaining (NDTR) */
    head = (uint16_t)(VN100_STM32_DMA_BUF - __HAL_DMA_GET_COUNTER(s->huart->hdmarx));
    pos  = s->last_pos;

    if (head == pos)
    {
        return;   /* no new data */
    }

    if (head > pos)
    {
        /* straight-line progress */
        vn100_rx_feed(s->vn, &s->dma_buf[pos], (uint16_t)(head - pos));
    }
    else
    {
        /* circular wrap-around: first to the end, then from the start to head */
        vn100_rx_feed(s->vn, &s->dma_buf[pos], (uint16_t)(VN100_STM32_DMA_BUF - pos));
        if (head > 0U)
        {
            vn100_rx_feed(s->vn, &s->dma_buf[0], head);
        }
    }

    s->last_pos = head;
}

void vn100_stm32_on_uart_idle(vn100_stm32_t *s)
{
    if (s == NULL)
    {
        return;
    }
    if (!__HAL_UART_GET_FLAG(s->huart, UART_FLAG_IDLE))
    {
        return;
    }
    __HAL_UART_CLEAR_IDLEFLAG(s->huart);
    /* Draining is NOT tied to IDLE alone: DMA half/full-transfer callbacks
       also call vn100_stm32_drain() (main.c), so under a continuous/
       high-Hz stream the ring never gets overwritten before being read,
       even if the line never goes idle, avoiding full-lap ambiguity. */
    vn100_stm32_drain(s);
}
