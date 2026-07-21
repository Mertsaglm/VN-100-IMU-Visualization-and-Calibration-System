/**
  ******************************************************************************
  * @file    vn100_cli.c
  * @brief   PC console application — proof of portability: runs the same
  *          core as the STM32 driver without touching STM32 hardware.
  ******************************************************************************
  * Reads a VN-100 stream (ASCII $VNYMR AND/OR binary 0xFA — dual mode) from
  * stdin, decodes it with the same core via vn100_rx_feed(), and prints the
  * decoded measurements.
  *
  * Hardware-free end-to-end demo (pipe in the Python simulator):
  *   cd pc && make
  *   python ../vn100_simulator.py --no-noise | build/vn100_cli        (ASCII)
  * Binary stream demo:
  *   python -c "import sys;from pyvn100 import binary;from pyvn100.simulator import \
  *     Vn100Simulator as S;s=S();[sys.stdout.buffer.write(binary.encode(s.sample(i*0.01,noise=False))) \
  *     for i in range(50)]" | build/vn100_cli
  ******************************************************************************
  */
#include <stdio.h>
#include <stdint.h>

#include "vn100.h"
#include "vn100_port_pc.h"

#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

static void on_packet(const vn100_data_t *d, void *user)
{
    (void)user;
    printf("YPR %+8.2f %+8.2f %+8.2f | gyro %+7.3f %+7.3f %+7.3f | acc %+6.2f %+6.2f %+6.2f\n",
           (double)d->yaw, (double)d->pitch, (double)d->roll,
           (double)d->gyro_x, (double)d->gyro_y, (double)d->gyro_z,
           (double)d->accel_x, (double)d->accel_y, (double)d->accel_z);
}

int main(void)
{
    vn100_t       v;
    vn100_port_t  port;
    vn100_pc_ctx_t ctx;
    uint8_t       buf[256];
    size_t        n;

#ifdef _WIN32
    /* Binary mode for stdin/stdout (otherwise Windows CRLF/0x1A translation
       corrupts the binary stream) */
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif

    vn100_port_pc_init(&port, &ctx, stdout);
    (void)vn100_init(&v, &port, on_packet, NULL, NULL);

    fprintf(stderr, "vn100_cli: reading VN-100 stream from stdin (ASCII+binary, dual mode)...\n");
    while ((n = fread(buf, 1U, sizeof(buf), stdin)) > 0U)
    {
        vn100_rx_feed(&v, buf, (uint16_t)n);
    }
    fprintf(stderr, "vn100_cli: done. packets=%u errors=%u bytes=%u\n",
            (unsigned)v.packet_count, (unsigned)v.error_count, (unsigned)v.byte_count);
    return 0;
}
