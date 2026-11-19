/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Adapted from TI's i2c_controller_rw_multibyte_fifo_poll example
 * for the MSPM0G3507 dev board:
 *   - I2C_0 on PA10 (SDA) / PA11 (SCL) -> LIS2HH12 accelerometer at 0x1E
 *   - Reads WHO_AM_I (register 0x0F, expected value 0x41)
 *   - Prints progress letters A-E over UART0 so we can see where it stops
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>

#define I2C_TX_PACKET_SIZE (1)
#define I2C_RX_PACKET_SIZE (1)
#define I2C_TARGET_ADDRESS (0x1E)   /* LIS2HH12, SA0 = GND */

uint8_t gTxPacket[I2C_TX_PACKET_SIZE] = {0x0F};   /* WHO_AM_I register */
volatile uint8_t gRxPacket[I2C_RX_PACKET_SIZE];

DL_I2C_ClockConfig gI2CclockConfig;
volatile uint32_t gClockSelFreq;
volatile uint32_t gDelayCycles;

static char line[80];

static void uart_puts(const char *s)
{
    while (*s) {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, *s++);
    }
}

/* Print the raw I2C controller status register (helps diagnose hangs) */
static void print_status(const char *label)
{
    snprintf(line, sizeof line, "%s (I2C status = 0x%08lX)\r\n", label,
             (unsigned long) DL_I2C_getControllerStatus(I2C_0_INST));
    uart_puts(line);
}

static void fail(const char *msg)
{
    uart_puts(msg);
    while (1) {
    }
}

int main(void)
{
    SYSCFG_DL_init();
    uart_puts("\r\nI2C test: reading LIS2HH12 WHO_AM_I...\r\n");

    /* I2C clock setup for the errata delay (unchanged from TI) */
    DL_I2C_getClockConfig(I2C_0_INST, &gI2CclockConfig);
    switch (gI2CclockConfig.clockSel) {
        case DL_I2C_CLOCK_BUSCLK:
            gClockSelFreq = 32000000;
            break;
        case DL_I2C_CLOCK_MFCLK:
            gClockSelFreq = 4000000;
            break;
        default:
            break;
    }
    gDelayCycles = (3 * (gI2CclockConfig.divideRatio + 1)) *
                   (CPUCLK_FREQ / gClockSelFreq);

    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &gTxPacket[0], I2C_TX_PACKET_SIZE);

    /* ---- A: wait for the controller to be idle ---- */
    print_status("A: waiting for idle");
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE))
        ;

    /* ---- B: write the register address ---- */
    uart_puts("B: sending register address 0x0F\r\n");
    DL_I2C_startControllerTransfer(I2C_0_INST, I2C_TARGET_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, I2C_TX_PACKET_SIZE);
    delay_cycles(gDelayCycles);   /* errata I2C_ERR_13 workaround */
    while (DL_I2C_getControllerStatus(I2C_0_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY)
        ;

    /* ---- C: write finished, check for NACK ---- */
    print_status("C: write done");
    if (DL_I2C_getControllerStatus(I2C_0_INST) &
        DL_I2C_CONTROLLER_STATUS_ERROR) {
        fail("I2C error: no ACK from 0x1E (check pins/address)\r\n");
    }
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE))
        ;
    delay_cycles(1000);

    /* ---- D: read 1 byte back ---- */
        /* ---- D: read 1 byte back (with timeout + status) ---- */
    uart_puts("D: reading 1 byte\r\n");
    DL_I2C_startControllerTransfer(I2C_0_INST, I2C_TARGET_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_RX, I2C_RX_PACKET_SIZE);
    delay_cycles(gDelayCycles);   /* same errata delay as the write */
    print_status("D1: read started");

    uint32_t timeout = 1000000;
    while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST) && --timeout)
        ;
    print_status("D2: after waiting");
    if (timeout == 0) {
        fail("Read timed out: no byte arrived in RX FIFO\r\n");
    }
    gRxPacket[0] = DL_I2C_receiveControllerData(I2C_0_INST);

    /* ---- E: done ---- */
    snprintf(line, sizeof line, "E: WHO_AM_I = 0x%02X (expect 0x41) -> %s\r\n",
             gRxPacket[0], gRxPacket[0] == 0x41 ? "Accelerometer found!"
                                                : "unexpected value");
    uart_puts(line);

    /* Keep re-printing so PuTTY can catch it any time */
    while (1) {
        delay_cycles(CPUCLK_FREQ);
        uart_puts(line);
    }
}