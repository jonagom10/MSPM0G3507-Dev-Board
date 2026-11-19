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
 * MSPM0G3507 dev board - LIS2HH12 accelerometer readout
 * (I2C sequence adapted from TI's i2c_controller_rw_multibyte_fifo_poll)
 *
 *   UART0: PA0 = TX, PA1 = RX -> CH340 -> USB serial, 115200 8N1
 *   I2C_0: PA10 = SDA, PA11 = SCL -> LIS2HH12 at 0x1E (SA0 = GND)
 *
 * 1. Checks WHO_AM_I (0x0F) == 0x41
 * 2. Enables the sensor: 100 Hz, X/Y/Z on, +/-2 g
 * 3. Prints X/Y/Z in milli-g five times per second
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* ---------------- LIS2HH12 ---------------- */
#define LIS2HH12_ADDR     (0x1E)
#define REG_WHO_AM_I      (0x0F)
#define REG_CTRL1         (0x20)
#define REG_OUT_X_L       (0x28)   /* X_L, X_H, Y_L, Y_H, Z_L, Z_H */
#define WHO_AM_I_VALUE    (0x41)

/* CTRL1 = 0b0011_1111: ODR = 100 Hz, BDU = 1, Z/Y/X enabled */
#define CTRL1_100HZ_XYZ   (0x3F)

/* +/-2 g full scale (default): 0.061 mg per LSB */
#define MG_PER_LSB_X1000  (61)

#define I2C_TIMEOUT       (1000000u)

/* ---------------- globals ---------------- */
static uint32_t gDelayCycles;   /* errata I2C_ERR_13 delay */
static char line[96];

/* ---------------- UART ---------------- */
static void uart_puts(const char *s)
{
    while (*s) {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, *s++);
    }
}

static void fail(const char *msg)
{
    uart_puts(msg);
    snprintf(line, sizeof line, "  (I2C status = 0x%08lX)\r\n",
             (unsigned long) DL_I2C_getControllerStatus(I2C_0_INST));
    uart_puts(line);
    while (1) {
    }
}

/* ---------------- I2C helpers ---------------- */

/* Same delay calculation as TI's example (errata I2C_ERR_13) */
static void i2c_setup_delay(void)
{
    DL_I2C_ClockConfig cfg;
    uint32_t clkFreq = 32000000;

    DL_I2C_getClockConfig(I2C_0_INST, &cfg);
    if (cfg.clockSel == DL_I2C_CLOCK_MFCLK) {
        clkFreq = 4000000;
    }
    gDelayCycles = (3 * (cfg.divideRatio + 1)) * (CPUCLK_FREQ / clkFreq);
}

static bool i2c_wait_idle(void)
{
    uint32_t t = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--t == 0) return false;
    }
    return true;
}

static bool i2c_wait_not_busy(void)
{
    uint32_t t = I2C_TIMEOUT;
    while (DL_I2C_getControllerStatus(I2C_0_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--t == 0) return false;
    }
    return true;
}

/* Write len bytes (max 8, the FIFO size). buf[0] is the register address. */
static bool i2c_write(const uint8_t *buf, uint8_t len)
{
    if (!i2c_wait_idle()) return false;

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, (uint8_t *) buf, len);
    DL_I2C_startControllerTransfer(I2C_0_INST, LIS2HH12_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);
    delay_cycles(gDelayCycles);

    if (!i2c_wait_not_busy()) return false;
    if (DL_I2C_getControllerStatus(I2C_0_INST) &
        DL_I2C_CONTROLLER_STATUS_ERROR) {
        return false;   /* NACK */
    }
    return i2c_wait_idle();
}

/* Read len bytes (max 8) into data. */
static bool i2c_read(uint8_t *data, uint8_t len)
{
    if (!i2c_wait_idle()) return false;

    DL_I2C_startControllerTransfer(I2C_0_INST, LIS2HH12_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    delay_cycles(gDelayCycles);

    for (uint8_t i = 0; i < len; i++) {
        uint32_t t = I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
            if (--t == 0) return false;
        }
        data[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }

    if (!i2c_wait_not_busy()) return false;
    return i2c_wait_idle();
}

/* ---------------- LIS2HH12 helpers ---------------- */
static bool lis_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_write(buf, 2);
}

/* Read consecutive registers. The LIS2HH12 auto-increments the register
 * address by default (CTRL4.IF_ADD_INC = 1). */
static bool lis_read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (!i2c_write(&reg, 1)) return false;   /* set register pointer */
    delay_cycles(1000);
    return i2c_read(data, len);
}

/* ---------------- main ---------------- */
int main(void)
{
    SYSCFG_DL_init();
    i2c_setup_delay();
    uart_puts("\r\nMSPM0 board: LIS2HH12 accelerometer\r\n");

    /* 1. Identify the sensor */
    uint8_t who = 0;
    if (!lis_read_regs(REG_WHO_AM_I, &who, 1)) {
        fail("I2C error reading WHO_AM_I\r\n");
    }
    snprintf(line, sizeof line, "WHO_AM_I = 0x%02X\r\n", who);
    uart_puts(line);
    if (who != WHO_AM_I_VALUE) {
        fail("Unexpected WHO_AM_I value\r\n");
    }

    /* 2. Power it on: 100 Hz, X/Y/Z enabled */
    if (!lis_write_reg(REG_CTRL1, CTRL1_100HZ_XYZ)) {
        fail("I2C error writing CTRL1\r\n");
    }

    /* Read CTRL1 back to confirm the write took */
    uint8_t ctrl1 = 0;
    if (lis_read_regs(REG_CTRL1, &ctrl1, 1)) {
        snprintf(line, sizeof line, "CTRL1 = 0x%02X (expect 0x3F)\r\n", ctrl1);
        uart_puts(line);
    }

    delay_cycles(CPUCLK_FREQ / 50);   /* ~20 ms for the first samples */

    /* 3. Stream X/Y/Z in milli-g */
    while (1) {
        uint8_t raw[6];

        if (lis_read_regs(REG_OUT_X_L, raw, 6)) {
            int16_t x = (int16_t) ((raw[1] << 8) | raw[0]);
            int16_t y = (int16_t) ((raw[3] << 8) | raw[2]);
            int16_t z = (int16_t) ((raw[5] << 8) | raw[4]);

            long x_mg = (long) x * MG_PER_LSB_X1000 / 1000;
            long y_mg = (long) y * MG_PER_LSB_X1000 / 1000;
            long z_mg = (long) z * MG_PER_LSB_X1000 / 1000;

            snprintf(line, sizeof line, "X = %6ld   Y = %6ld   Z = %6ld  mg\r\n",
                     x_mg, y_mg, z_mg);
            uart_puts(line);
        } else {
            uart_puts("I2C read error\r\n");
        }

        delay_cycles(CPUCLK_FREQ / 5);   /* ~200 ms */
    }
}