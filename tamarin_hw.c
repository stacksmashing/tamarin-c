/*  
    Tamarin-C - Tool for exploring USB-C on iPhones & macs
    Copyright 2023 Thomas 'stacksmashing' Roth - code@stacksmashing.net

    Heavily based on previous work by the AsahiLinux project, Marc Zyngier, etc.
    Check out: vdmtool and macvdmtool

    Released under GPLv3. See LICENSE for details.
*/

#include "tamarin_hw.h"

#include "tusb.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

const uint PIN_SHIFTER_SUPPLY = 0;
const uint PIN_SBU1_DIR = 1;
const uint PIN_SBU2_DIR = 2;
const uint PIN_USB_DP_DIR = 5;
const uint PIN_USB_DN_DIR = 6;
const uint PIN_VBUS_EN = 3;
const uint PIN_VBUS_FLG = 4;
const uint PIN_USB_DP = 5;
const uint PIN_USB_DN = 6;
const uint PIN_SBU1 = 12;
const uint PIN_SBU2 = 13;
const uint PIN_I2C_SDA = 16;
const uint PIN_I2C_SCL = 17;
const uint PIN_FUSB_INT = 18;
i2c_inst_t * FUSB_I2C_INST = i2c0;
const uint8_t FUSB_I2C_ADDR = 0x22;


const int SHIFTER_DIRECTION_OUT = 1;
const int SHIFTER_DIRECTION_IN = 0;

uint PROBE_PIN_SWCLK_DIR = PIN_SBU1_DIR;
uint PROBE_PIN_SWCLK = PIN_SBU1;
uint PROBE_PIN_SWDIO_DIR = PIN_SBU2_DIR;
uint PROBE_PIN_SWDIO = PIN_SBU2;

void vuprintf(const char* format, va_list args) {
    char buf[128];
    vsnprintf(buf, 128, format, args);
    tud_cdc_n_write_str(ITF_CONSOLE, buf);
    tud_cdc_n_write_flush(ITF_CONSOLE);
    tud_task();
}

void uprintf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vuprintf(format, args);
    va_end(args);
}
