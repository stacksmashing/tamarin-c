/* Copyright 2015 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* USB Power delivery port management - common header for TCPM drivers */

#ifndef __CROS_EC_USB_PD_TCPM_TCPM_H
#define __CROS_EC_USB_PD_TCPM_TCPM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tcpm_driver.h"
#include "usb_pd_tcpm.h"

#if defined(CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE) && \
	!defined(CONFIG_USB_PD_DUAL_ROLE)
#error "DRP auto toggle requires board to have DRP support"
#error "Please upgrade your board configuration"
#endif

#ifndef CONFIG_USB_PD_TCPC
extern const struct tcpc_config_t tcpc_config[];

/* I2C wrapper functions - get I2C port / slave addr from config struct. */
int16_t tcpc_write(int16_t port, int16_t reg, int16_t val);
int16_t tcpc_write16(int16_t port, int16_t reg, int16_t val);
int16_t tcpc_read(int16_t port, int16_t reg, int16_t *val);
int16_t tcpc_read16(int16_t port, int16_t reg, int16_t *val);
int16_t tcpc_xfer(int16_t port,
		const uint8_t *out, int16_t out_size,
		uint8_t *in, int16_t in_size,
		int16_t flags);

#endif

#ifdef __cplusplus
}
#endif

#endif
