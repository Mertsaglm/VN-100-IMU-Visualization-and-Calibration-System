/**
  ******************************************************************************
  * @file    vn100_port_pc.h
  * @brief   PC adaptation of the VN-100 PORT interface (LOW — PC-SPECIFIC)
  ******************************************************************************
  * PC counterpart of vn100_port_stm32.c: write() writes to a FILE* stream
  * (stdout or a serial port fd), millis() is the clock source, and the
  * critical section is unnecessary on PC (single-threaded), hence NULL.
  ******************************************************************************
  */
#ifndef VN100_PORT_PC_H
#define VN100_PORT_PC_H

#include <stdio.h>
#include "vn100_port.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    FILE *out;   /**< TX stream to the sensor (stdout / serial port) */
} vn100_pc_ctx_t;

/** Sets up a vn100_port_t as a PC port (out: TX destination). */
void vn100_port_pc_init(vn100_port_t *port, vn100_pc_ctx_t *ctx, FILE *out);

#ifdef __cplusplus
}
#endif

#endif /* VN100_PORT_PC_H */
