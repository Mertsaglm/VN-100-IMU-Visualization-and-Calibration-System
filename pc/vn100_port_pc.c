/**
  ******************************************************************************
  * @file    vn100_port_pc.c
  * @brief   PC adaptation of the VN-100 PORT interface (PC-SPECIFIC)
  ******************************************************************************
  * Symmetric with the STM32 port (vn100_port_stm32.c): the one
  * platform-specific file that lets the same core (vn100.c/protocol/binary)
  * run on PC.
  ******************************************************************************
  */
#include "vn100_port_pc.h"
#include <time.h>

static int pc_write(void *ctx, const uint8_t *data, uint16_t len)
{
    vn100_pc_ctx_t *c = (vn100_pc_ctx_t *)ctx;
    FILE *f = ((c != NULL) && (c->out != NULL)) ? c->out : stdout;
    size_t w = fwrite(data, 1U, (size_t)len, f);
    (void)fflush(f);
    return (w == (size_t)len) ? (int)w : -1;
}

static uint32_t pc_millis(void *ctx)
{
    (void)ctx;
    return (uint32_t)(((unsigned long long)clock() * 1000ULL) / (unsigned long long)CLOCKS_PER_SEC);
}

void vn100_port_pc_init(vn100_port_t *port, vn100_pc_ctx_t *ctx, FILE *out)
{
    ctx->out = out;
    port->write          = pc_write;
    port->millis         = pc_millis;
    port->enter_critical = NULL;   /* PC is single-threaded -> critical section unneeded */
    port->exit_critical  = NULL;
    port->ctx            = ctx;
}
