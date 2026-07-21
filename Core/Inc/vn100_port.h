/**
  ******************************************************************************
  * @file    vn100_port.h
  * @brief   VN-100 PORT interface - the "seam" of the abstraction (LOW-level)
  ******************************************************************************
  * This struct is the single point connecting the core to a platform. Porting
  * to a new board or to PC only requires writing a port file that implements
  * this interface; the protocol and high-level API layers stay unchanged.
  *
  *   write()              : send bytes to the sensor (STM32: UART; PC: serial)
  *   millis()              : time source in ms
  *   enter/exit_critical    : atomic access to shared data (may be NULL)
  ******************************************************************************
  */
#ifndef VN100_PORT_H
#define VN100_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct vn100_port
{
    int      (*write)(void *ctx, const uint8_t *data, uint16_t len);
    uint32_t (*millis)(void *ctx);
    void     (*enter_critical)(void *ctx);  /**< may be NULL */
    void     (*exit_critical)(void *ctx);   /**< may be NULL */
    void      *ctx;                         /**< platform-specific context (huart* etc.) */
} vn100_port_t;

#ifdef __cplusplus
}
#endif

#endif /* VN100_PORT_H */
