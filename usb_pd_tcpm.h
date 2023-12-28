/* Copyright 2015 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* USB Power delivery port management */

#ifndef __CROS_EC_USB_PD_TCPM_H
#define __CROS_EC_USB_PD_TCPM_H

#ifdef __cplusplus
extern "C" {
#endif

/* List of common error codes that can be returned */
	enum ec_error_list {
		/* Success - no error */
		EC_SUCCESS = 0,
		/* Unknown error */
		EC_ERROR_UNKNOWN = 1,
		/* Function not implemented yet */
		EC_ERROR_UNIMPLEMENTED = 2,
		/* Overflow error; too much input provided. */
		EC_ERROR_OVERFLOW = 3,
		/* Timeout */
		EC_ERROR_TIMEOUT = 4,
		/* Invalid argument */
		EC_ERROR_INVAL = 5,
		/* Already in use, or not ready yet */
		EC_ERROR_BUSY = 6,
		/* Access denied */
		EC_ERROR_ACCESS_DENIED = 7,
		/* Failed because component does not have power */
		EC_ERROR_NOT_POWERED = 8,
		/* Failed because component is not calibrated */
		EC_ERROR_NOT_CALIBRATED = 9,
		/* Failed because CRC error */
		EC_ERROR_CRC = 10,
	};

/* Flags for i2c_xfer() */
#define I2C_XFER_START (1 << 0)	/* Start smbus session from idle state */
#define I2C_XFER_STOP (1 << 1)	/* Terminate smbus session with stop bit */
#define I2C_XFER_SINGLE (I2C_XFER_START | I2C_XFER_STOP)	/* One transaction */

/* Default retry count for transmitting */
#define PD_RETRY_COUNT 3

/* Time to wait for TCPC to complete transmit */
#define PD_T_TCPC_TX_TIMEOUT  (100*MSEC_US)

/* No connect voltage threshold for sources based on Rp */
#define PD_SRC_DEF_VNC_MV        1600
#define PD_SRC_1_5_VNC_MV        1600
#define PD_SRC_3_0_VNC_MV        2600

/* Rd voltage threshold for sources based on Rp */
#define PD_SRC_DEF_RD_THRESH_MV  200
#define PD_SRC_1_5_RD_THRESH_MV  400
#define PD_SRC_3_0_RD_THRESH_MV  800

/* Control Message type */
	enum pd_ctrl_msg_type {
		/* 0 Reserved */
		PD_CTRL_GOOD_CRC = 1,
		PD_CTRL_GOTO_MIN = 2,
		PD_CTRL_ACCEPT = 3,
		PD_CTRL_REJECT = 4,
		PD_CTRL_PING = 5,
		PD_CTRL_PS_RDY = 6,
		PD_CTRL_GET_SOURCE_CAP = 7,
		PD_CTRL_GET_SINK_CAP = 8,
		PD_CTRL_DR_SWAP = 9,
		PD_CTRL_PR_SWAP = 10,
		PD_CTRL_VCONN_SWAP = 11,
		PD_CTRL_WAIT = 12,
		PD_CTRL_SOFT_RESET = 13,
		/* 14-15 Reserved */

		/* Used for REV 3.0 */
		PD_CTRL_NOT_SUPPORTED = 16,
		PD_CTRL_GET_SOURCE_CAP_EXT = 17,
		PD_CTRL_GET_STATUS = 18,
		PD_CTRL_FR_SWAP = 19,
		PD_CTRL_GET_PPS_STATUS = 20,
		PD_CTRL_GET_COUNTRY_CODES = 21,
		/* 22-31 Reserved */
	};

/* Extended message type for REV 3.0 */
	enum pd_ext_msg_type {
		/* 0 Reserved */
		PD_EXT_SOURCE_CAP = 1,
		PD_EXT_STATUS = 2,
		PD_EXT_GET_BATTERY_CAP = 3,
		PD_EXT_GET_BATTERY_STATUS = 4,
		PD_EXT_BATTERY_CAP = 5,
		PD_EXT_GET_MANUFACTURER_INFO = 6,
		PD_EXT_MANUFACTURER_INFO = 7,
		PD_EXT_SECURITY_REQUEST = 8,
		PD_EXT_SECURITY_RESPONSE = 9,
		PD_EXT_FIRMWARE_UPDATE_REQUEST = 10,
		PD_EXT_FIRMWARE_UPDATE_RESPONSE = 11,
		PD_EXT_PPS_STATUS = 12,
		PD_EXT_COUNTRY_INFO = 13,
		PD_EXT_COUNTRY_CODES = 14,
		/* 15-31 Reserved */
	};

/* Data message type */
	enum pd_data_msg_type {
		/* 0 Reserved */
		PD_DATA_SOURCE_CAP = 1,
		PD_DATA_REQUEST = 2,
		PD_DATA_BIST = 3,
		PD_DATA_SINK_CAP = 4,
		/* 5-14 Reserved for REV 2.0 */
		PD_DATA_BATTERY_STATUS = 5,
		PD_DATA_ALERT = 6,
		PD_DATA_GET_COUNTRY_INFO = 7,
		/* 8-14 Reserved for REV 3.0 */
		PD_DATA_VENDOR_DEF = 15,
	};
/* build extended message header */
/* All extended messages are chunked, so set bit 15 */
#define PD_EXT_HEADER(cnum, rchk, dsize) \
   ((1 << 15) | ((cnum) << 11) | \
   ((rchk) << 10) | (dsize))

/* build message header */
#define PD_HEADER(type, prole, drole, id, cnt, rev, ext) \
  ((type) | ((rev) << 6) | \
  ((drole) << 5) | ((prole) << 8) | \
  ((id) << 9) | ((cnt) << 12) | ((ext) << 15))

/* Used for processing pd header */
#define PD_HEADER_EXT(header)  (((header) >> 15) & 1)
#define PD_HEADER_CNT(header)  (((header) >> 12) & 7)
#define PD_HEADER_TYPE(header) ((header) & 0x1F)
#define PD_HEADER_ID(header)   (((header) >> 9) & 7)
#define PD_HEADER_REV(header)  (((header) >> 6) & 3)

/* Used for processing pd extended header */
#define PD_EXT_HEADER_CHUNKED(header)   (((header) >> 15) & 1)
#define PD_EXT_HEADER_CHUNK_NUM(header) (((header) >> 11) & 0xf)
#define PD_EXT_HEADER_REQ_CHUNK(header) (((header) >> 10) & 1)
#define PD_EXT_HEADER_DATA_SIZE(header) ((header) & 0x1ff)

#define PD_REV10 0
#define PD_REV20 1
#define PD_REV30 2

	enum tcpc_cc_voltage_status {
		TYPEC_CC_VOLT_OPEN = 0,
		TYPEC_CC_VOLT_RA = 1,
		TYPEC_CC_VOLT_RD = 2,
		TYPEC_CC_VOLT_SNK_DEF = 5,
		TYPEC_CC_VOLT_SNK_1_5 = 6,
		TYPEC_CC_VOLT_SNK_3_0 = 7,
	};

	enum tcpc_cc_pull {
		TYPEC_CC_RA = 0,
		TYPEC_CC_RP = 1,
		TYPEC_CC_RD = 2,
		TYPEC_CC_OPEN = 3,
	};

    // RP = Pull-up resistor as used on the CC line
	enum tcpc_rp_value {
		TYPEC_RP_USB = 0,
		TYPEC_RP_1A5 = 1,
		TYPEC_RP_3A0 = 2,
		TYPEC_RP_RESERVED = 3,
	};

	enum tcpm_transmit_type {
		TCPC_TX_SOP = 0,
		TCPC_TX_SOP_PRIME = 1,
		TCPC_TX_SOP_PRIME_PRIME = 2,
		TCPC_TX_SOP_DEBUG_PRIME = 3,
		TCPC_TX_SOP_DEBUG_PRIME_PRIME = 4,
		TCPC_TX_HARD_RESET = 5,
		TCPC_TX_CABLE_RESET = 6,
		TCPC_TX_BIST_MODE_2 = 7
	};

	enum tcpc_transmit_complete {
		TCPC_TX_COMPLETE_SUCCESS = 0,
		TCPC_TX_COMPLETE_DISCARDED = 1,
		TCPC_TX_COMPLETE_FAILED = 2,
	};

	struct tcpc_config_t {
		int i2c_host_port;
		int i2c_slave_addr;
		const struct tcpm_drv *drv;
	};

#ifdef __cplusplus
}
#endif
#endif				/* __CROS_EC_USB_PD_TCPM_H */
