/*  
    Tamarin-C - Tool for exploring USB-C on iPhones & macs
    Copyright 2023 Thomas 'stacksmashing' Roth - code@stacksmashing.net

    Heavily based on previous work by the AsahiLinux project, Marc Zyngier, etc.
    Check out: vdmtool and macvdmtool

    Released under GPLv3. See LICENSE for details.
*/

#pragma once

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"

#include <stdarg.h>

extern const uint PIN_SHIFTER_SUPPLY;
extern const uint PIN_SBU1_DIR;
extern const uint PIN_SBU2_DIR;
extern const uint PIN_USB_DP_DIR;
extern const uint PIN_USB_DN_DIR;
extern const uint PIN_VBUS_EN;
extern const uint PIN_VBUS_FLG;
extern const uint PIN_SBU1;
extern const uint PIN_SBU2;
extern const uint PIN_USB_DP;
extern const uint PIN_USB_DN;
extern const uint PIN_I2C_SDA;
extern const uint PIN_I2C_SCL;
extern const uint PIN_FUSB_INT;
extern i2c_inst_t * FUSB_I2C_INST;
extern const uint8_t FUSB_I2C_ADDR;

extern const int SHIFTER_DIRECTION_OUT;
extern const int SHIFTER_DIRECTION_IN;

// PIO config
#define PROBE_PIO pio0
#define PROBE_SM 0
extern uint PROBE_PIN_SWCLK_DIR;
extern uint PROBE_PIN_SWDIO_DIR;
extern uint PROBE_PIN_SWCLK;
extern uint PROBE_PIN_SWDIO;

#define ITF_CONSOLE 0
#define ITF_DCSD 1
#define ITF_JTAG 2

void vuprintf(const char* format, va_list args);
void uprintf(const char* format, ...);

#if true
#define tamarin_info(format,args...) uprintf(format, ## args)
#else
#define tamarin_info(format,...) ((void)0)
#endif


#if true
#define tamarin_debug(format,args...) uprintf(format, ## args)
#else
#define tamarin_debug(format,...) ((void)0)
#endif

#if true
#define tamarin_dump(format,args...) uprintf(format, ## args)
#else
#define tamarin_dump(format,...) ((void)0)
#endif