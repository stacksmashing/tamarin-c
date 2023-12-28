/*  
    FUSB302.c - Library for interacting with the FUSB302B chip.
    Copyright 2015 The Chromium OS Authors
    Copyright 2017 Jason Cerundolo
	Modified in 2023 by Thomas 'stacksmashing' Roth
    Released under an MIT license.
*/

#include <stdint.h>
#include <string.h>

#include "FUSB302.h"
#include "usb_pd_tcpm.h"
#include "tcpm_driver.h"
#include "platform.h"

#define PACKET_IS_GOOD_CRC(head) (PD_HEADER_TYPE(head) == PD_CTRL_GOOD_CRC && \
				  PD_HEADER_CNT(head) == 0)

static struct fusb302_chip_state {
	int16_t cc_polarity;
	int16_t vconn_enabled;
	/* 1 = pulling up (DFP) 0 = pulling down (UFP) */
	int16_t pulling_up;
	int16_t rx_enable;
	uint8_t control1;
	uint8_t mdac_vnc;
	uint8_t mdac_rd;
	uint8_t msgid;
} state[CONFIG_USB_PD_PORT_COUNT];

int get_polarity() {
	return state[0].cc_polarity;
}

/*
 * Bring the FUSB302 out of reset after Hard Reset signaling. This will
 * automatically flush both the Rx and Tx FIFOs.
 */
void fusb302_pd_reset(int16_t port)
{
	tcpc_write(port, TCPC_REG_RESET, TCPC_REG_RESET_PD_RESET);
	state[port].msgid = 0;
}

/*
 * Flush our Rx FIFO. To prevent packet framing issues, this function should
 * only be called when Rx is disabled.
 */
void fusb302_flush_rx_fifo(int16_t port)
{
	tcpc_write(port, TCPC_REG_CONTROL1,
		   state[port].control1 | TCPC_REG_CONTROL1_RX_FLUSH);
}

void fusb302_flush_tx_fifo(int16_t port)
{
	int16_t reg;

	tcpc_read(port, TCPC_REG_CONTROL0, &reg);
	reg |= TCPC_REG_CONTROL0_TX_FLUSH;
	tcpc_write(port, TCPC_REG_CONTROL0, reg);
}

void fusb302_auto_goodcrc_enable(int16_t port, int16_t enable)
{
	int16_t reg;

	tcpc_read(port, TCPC_REG_SWITCHES1, &reg);

	if (enable)
		reg |= TCPC_REG_SWITCHES1_AUTO_GCRC;
	else
		reg &= ~TCPC_REG_SWITCHES1_AUTO_GCRC;

	// Spec says these should be zero, default is bad
	reg &= ~(TCPC_REG_SWITCHES1_SPECREV0 | TCPC_REG_SWITCHES1_SPECREV1);

	tcpc_write(port, TCPC_REG_SWITCHES1, reg);
}

/* Convert BC LVL values (in FUSB302) to Type-C CC Voltage Status */
static int16_t convert_bc_lvl(int16_t port, int16_t bc_lvl)
{
	/* assume OPEN unless one of the following conditions is true... */
	int16_t ret = TYPEC_CC_VOLT_OPEN;

	if (state[port].pulling_up) {
		if (bc_lvl == 0x00)
			ret = TYPEC_CC_VOLT_RA;
		else if (bc_lvl < 0x3)
			ret = TYPEC_CC_VOLT_RD;
	} else {
		if (bc_lvl == 0x1)
			ret = TYPEC_CC_VOLT_SNK_DEF;
		else if (bc_lvl == 0x2)
			ret = TYPEC_CC_VOLT_SNK_1_5;
		else if (bc_lvl == 0x3)
			ret = TYPEC_CC_VOLT_SNK_3_0;
	}

	return ret;
}


int16_t fusb302_measure_cc_pin_source(int16_t port, int16_t cc_to_measure)
{
	int16_t switches0_reg;
	int16_t reg;
	int16_t cc_lvl;

	/* Read status register */
	tcpc_read(port, TCPC_REG_SWITCHES0, &reg);

	/* Save current value */
	switches0_reg = reg;

	/* Clear pull-up register settings and measure bits */
	reg &= ~(TCPC_REG_SWITCHES0_MEAS_CC1 | TCPC_REG_SWITCHES0_MEAS_CC2 |
                TCPC_REG_SWITCHES0_CC1_PU_EN | TCPC_REG_SWITCHES0_CC2_PU_EN);
	
	
	/* Set desired pullup register bit */
	if (cc_to_measure == 0)
		reg |= TCPC_REG_SWITCHES0_CC1_PU_EN;
	else
		reg |= TCPC_REG_SWITCHES0_CC2_PU_EN;
	/* Set CC measure bit */
	if(cc_to_measure == 0) { // cc1
		reg |= TCPC_REG_SWITCHES0_MEAS_CC1;
	} else {
		reg |= TCPC_REG_SWITCHES0_MEAS_CC2;
	}

	/* Set measurement switch */
	tcpc_write(port, TCPC_REG_SWITCHES0, reg);

	// one cc line goes to 1.8, one to 1.6...
	//uint16_t measure_reg = TCPC_REG_MEASURE_MDAC_MV(1700);
	uint16_t measure_reg = TCPC_REG_MEASURE_MDAC_MV(2400);

	/* Set MDAC for Open vs Rd/Ra comparison */
	tcpc_write(port, TCPC_REG_MEASURE, measure_reg);

	/* Wait on measurement */
	platform_usleep(250);

	/* Read status register */
	tcpc_read(port, TCPC_REG_STATUS0, &reg);

	/* Assume open */
	cc_lvl = TYPEC_CC_VOLT_OPEN;

	uint16_t ret = 0;
	/* CC level is below the 'no connect' threshold (vOpen) */
	if ((reg & TCPC_REG_STATUS0_COMP) == 0) {
		ret = 0;
	} else {
		ret = 1;
	}
	// 	/* Set MDAC for Rd vs Ra comparison */
	// 	tcpc_write(port, TCPC_REG_MEASURE, state[port].mdac_rd);

	// 	/* Wait on measurement */
	// 	platform_usleep(250);

	// 	/* Read status register */
	// 	tcpc_read(port, TCPC_REG_STATUS0, &reg);

	// 	cc_lvl = (reg & TCPC_REG_STATUS0_COMP) ? TYPEC_CC_VOLT_RD
	// 	    : TYPEC_CC_VOLT_RA;
	// }

	/* Restore SWITCHES0 register to its value prior */
	tcpc_write(port, TCPC_REG_SWITCHES0, switches0_reg);

	// return cc_lvl;
	return ret;
}

static int16_t measure_cc_pin_source(int16_t port, int16_t cc_measure)
{
	int16_t switches0_reg;
	int16_t reg;
	int16_t cc_lvl;

	/* Read status register */
	tcpc_read(port, TCPC_REG_SWITCHES0, &reg);
	/* Save current value */
	switches0_reg = reg;
	/* Clear pull-up register settings and measure bits */
	reg &= ~(TCPC_REG_SWITCHES0_MEAS_CC1 | TCPC_REG_SWITCHES0_MEAS_CC2);
	/* Set desired pullup register bit */
	// if (cc_measure == TCPC_REG_SWITCHES0_MEAS_CC1)
	// 	reg |= TCPC_REG_SWITCHES0_CC1_PU_EN;
	// else
	// 	reg |= TCPC_REG_SWITCHES0_CC2_PU_EN;
	/* Set CC measure bit */
	reg |= cc_measure;

	/* Set measurement switch */
	tcpc_write(port, TCPC_REG_SWITCHES0, reg);

	/* Set MDAC for Open vs Rd/Ra comparison */
	tcpc_write(port, TCPC_REG_MEASURE, state[port].mdac_vnc);

	/* Wait on measurement */
	platform_usleep(250);

	/* Read status register */
	tcpc_read(port, TCPC_REG_STATUS0, &reg);

	/* Assume open */
	cc_lvl = TYPEC_CC_VOLT_OPEN;

	/* CC level is below the 'no connect' threshold (vOpen) */
	if ((reg & TCPC_REG_STATUS0_COMP) == 0) {
		/* Set MDAC for Rd vs Ra comparison */
		tcpc_write(port, TCPC_REG_MEASURE, state[port].mdac_rd);

		/* Wait on measurement */
		platform_usleep(250);

		/* Read status register */
		tcpc_read(port, TCPC_REG_STATUS0, &reg);

		cc_lvl = (reg & TCPC_REG_STATUS0_COMP) ? TYPEC_CC_VOLT_RD
		    : TYPEC_CC_VOLT_RA;
	}

	/* Restore SWITCHES0 register to its value prior */
	tcpc_write(port, TCPC_REG_SWITCHES0, switches0_reg);

	return cc_lvl;
}

/* Determine cc pin state for source when in manual detect mode */
static void detect_cc_pin_source_manual(int16_t port, int16_t *cc1_lvl, int16_t *cc2_lvl)
{
	int16_t cc1_measure = TCPC_REG_SWITCHES0_MEAS_CC1;
	int16_t cc2_measure = TCPC_REG_SWITCHES0_MEAS_CC2;

	if (state[port].vconn_enabled) {
		/* If VCONN enabled, measure cc_pin that matches polarity */
		if (state[port].cc_polarity)
			*cc2_lvl = measure_cc_pin_source(port, cc2_measure);
		else
			*cc1_lvl = measure_cc_pin_source(port, cc1_measure);
	} else {
		/* If VCONN not enabled, measure both cc1 and cc2 */
		*cc1_lvl = measure_cc_pin_source(port, cc1_measure);
		*cc2_lvl = measure_cc_pin_source(port, cc2_measure);
	}

}

/* Determine cc pin state for sink */
static void detect_cc_pin_sink(int16_t port, int16_t *cc1, int16_t *cc2)
{
	int16_t reg;
	int16_t orig_meas_cc1;
	int16_t orig_meas_cc2;
	int16_t bc_lvl_cc1;
	int16_t bc_lvl_cc2;

	/*
	 * Measure CC1 first.
	 */
	tcpc_read(port, TCPC_REG_SWITCHES0, &reg);

	/* save original state to be returned to later... */
	if (reg & TCPC_REG_SWITCHES0_MEAS_CC1)
		orig_meas_cc1 = 1;
	else
		orig_meas_cc1 = 0;

	if (reg & TCPC_REG_SWITCHES0_MEAS_CC2)
		orig_meas_cc2 = 1;
	else
		orig_meas_cc2 = 0;

	/* Disable CC2 measurement switch, enable CC1 measurement switch */
	reg &= ~TCPC_REG_SWITCHES0_MEAS_CC2;
	reg |= TCPC_REG_SWITCHES0_MEAS_CC1;

	tcpc_write(port, TCPC_REG_SWITCHES0, reg);

	/* CC1 is now being measured by FUSB302. */

	/* Wait on measurement */
	platform_usleep(250);

	tcpc_read(port, TCPC_REG_STATUS0, &bc_lvl_cc1);

	/* mask away unwanted bits */
	bc_lvl_cc1 &= (TCPC_REG_STATUS0_BC_LVL0 | TCPC_REG_STATUS0_BC_LVL1);

	/*
	 * Measure CC2 next.
	 */

	tcpc_read(port, TCPC_REG_SWITCHES0, &reg);

	/* Disable CC1 measurement switch, enable CC2 measurement switch */
	reg &= ~TCPC_REG_SWITCHES0_MEAS_CC1;
	reg |= TCPC_REG_SWITCHES0_MEAS_CC2;

	tcpc_write(port, TCPC_REG_SWITCHES0, reg);

	/* CC2 is now being measured by FUSB302. */

	/* Wait on measurement */
	platform_usleep(250);

	tcpc_read(port, TCPC_REG_STATUS0, &bc_lvl_cc2);

	/* mask away unwanted bits */
	bc_lvl_cc2 &= (TCPC_REG_STATUS0_BC_LVL0 | TCPC_REG_STATUS0_BC_LVL1);

	*cc1 = convert_bc_lvl(port, bc_lvl_cc1);
	*cc2 = convert_bc_lvl(port, bc_lvl_cc2);

	/* return MEAS_CC1/2 switches to original state */
	tcpc_read(port, TCPC_REG_SWITCHES0, &reg);
	if (orig_meas_cc1)
		reg |= TCPC_REG_SWITCHES0_MEAS_CC1;
	else
		reg &= ~TCPC_REG_SWITCHES0_MEAS_CC1;
	if (orig_meas_cc2)
		reg |= TCPC_REG_SWITCHES0_MEAS_CC2;
	else
		reg &= ~TCPC_REG_SWITCHES0_MEAS_CC2;

	tcpc_write(port, TCPC_REG_SWITCHES0, reg);

}

/* Parse header bytes for the size of packet */
static int16_t get_num_bytes(uint16_t header)
{
	int16_t rv;

	/* Grab the Number of Data Objects field. */
	rv = PD_HEADER_CNT(header);

	/* Multiply by four to go from 32-bit words -> bytes */
	rv *= 4;

	/* Plus 2 for header */
	rv += 2;

	return rv;
}

static int16_t fusb302_send_message(int16_t port, uint16_t header,
				const uint32_t * data, uint8_t * buf,
				int16_t buf_pos)
{
	int16_t rv;
	int16_t reg;
	int16_t len;

	len = get_num_bytes(header);

	/*
	 * packsym tells the TXFIFO that the next X bytes are payload,
	 * and should not be interpreted as special tokens.
	 * The 5 LSBs represent X, the number of bytes.
	 */
	reg = fusb302_TKN_PACKSYM;
	reg |= (len & 0x1F);

	buf[buf_pos++] = reg;

	/* write in the header */
	reg = header;
	buf[buf_pos++] = reg & 0xFF;

	reg >>= 8;
	buf[buf_pos++] = reg & 0xFF;

	/* header is done, subtract from length to make this for-loop simpler */
	len -= 2;

	/* write data objects, if present */
	memcpy(&buf[buf_pos], data, len);
	buf_pos += len;

	/* put in the CRC */
	buf[buf_pos++] = fusb302_TKN_JAMCRC;

	/* put in EOP */
	buf[buf_pos++] = fusb302_TKN_EOP;

	/* Turn transmitter off after sending message */
	buf[buf_pos++] = fusb302_TKN_TXOFF;

	/* Start transmission */
	reg = fusb302_TKN_TXON;
	buf[buf_pos++] = fusb302_TKN_TXON;

	/* burst write for speed! */
	rv = tcpc_xfer(port, buf, buf_pos, 0, 0, I2C_XFER_SINGLE);

	return rv;
}


void fusb302_setup_mdac() {
	// INterrupt when crossing 1400V.
	
	uint16_t reg = TCPC_REG_MEASURE_MDAC_MV(1400);
	tcpc_write(0, TCPC_REG_MEASURE, reg);
}

void fusb302_setup_toggle() {
	// tcpc_read(port, TCPC_REG_CONTROL2, &reg);
	// reg &= ~TCPC_REG_CONTROL2_TOGGLE;
	uint8_t reg = (TCPC_REG_CONTROL2_MODE_DFP << 1);
	// reg = 7 | 0b100000;
	tcpc_write(0, TCPC_REG_CONTROL2, reg);
}

void fusb302_enable_toggle() {
	uint16_t reg;
	tcpc_read(0, TCPC_REG_CONTROL2, &reg);
	reg |= TCPC_REG_CONTROL2_TOGGLE;
	// reg &= ~TCPC_REG_CONTROL2_TOGGLE;
	// uint8_t reg = (TCPC_REG_CONTROL2_MODE_DFP << 1);
	// reg = 7 | 0b100000;
	tcpc_write(0, TCPC_REG_CONTROL2, reg);
}

void fusb302_disable_toggle() {
	uint16_t reg;
	tcpc_read(0, TCPC_REG_CONTROL2, &reg);
	reg &= ~TCPC_REG_CONTROL2_TOGGLE;
	// reg &= ~TCPC_REG_CONTROL2_TOGGLE;
	// uint8_t reg = (TCPC_REG_CONTROL2_MODE_DFP << 1);
	// reg = 7 | 0b100000;
	tcpc_write(0, TCPC_REG_CONTROL2, reg);
}

int16_t fusb302_tcpm_select_rp_value(int16_t port, int16_t rp)
{
	int16_t reg;
	int16_t rv;
	uint8_t vnc, rd;

	rv = tcpc_read(port, TCPC_REG_CONTROL0, &reg);
	if (rv)
		return rv;

	/* Set the current source for Rp value */
	reg &= ~TCPC_REG_CONTROL0_HOST_CUR_MASK;
	switch (rp) {
	case TYPEC_RP_1A5:
		reg |= TCPC_REG_CONTROL0_HOST_CUR_1A5;
		vnc = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_1_5_VNC_MV);
		rd = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_1_5_RD_THRESH_MV);
		break;
	case TYPEC_RP_3A0:
		reg |= TCPC_REG_CONTROL0_HOST_CUR_3A0;
		vnc = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_3_0_VNC_MV);
		rd = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_3_0_RD_THRESH_MV);
		break;
	case TYPEC_RP_USB:
	default:
		reg |= TCPC_REG_CONTROL0_HOST_CUR_USB;
		vnc = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_VNC_MV);
		rd = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_RD_THRESH_MV);
	}
	state[port].mdac_vnc = vnc;
	state[port].mdac_rd = rd;

	uint8_t value = reg;
	// printf("%sTCPC_REG_CONTROL0_TX_FLUSH\n", (value & TCPC_REG_CONTROL0_TX_FLUSH) ? "" : "!");
    // printf("%sTCPC_REG_CONTROL0_INT_MASK\n", (value & TCPC_REG_CONTROL0_INT_MASK) ? "" : "!");
    
    // // Handle HOST_CUR bits separately
    // switch (value & (3<<2)) {
    //     case TCPC_REG_CONTROL0_HOST_CUR_3A0:
    //         printf("TCPC_REG_CONTROL0_HOST_CUR_3A0\n");
    //         break;
    //     case TCPC_REG_CONTROL0_HOST_CUR_1A5:
    //         printf("TCPC_REG_CONTROL0_HOST_CUR_1A5\n");
    //         break;
    //     case TCPC_REG_CONTROL0_HOST_CUR_USB:
    //         printf("TCPC_REG_CONTROL0_HOST_CUR_USB\n");
    //         break;
    //     default:
    //         printf("!TCPC_REG_CONTROL0_HOST_CUR_3A0\n");
    //         printf("!TCPC_REG_CONTROL0_HOST_CUR_1A5\n");
    //         printf("!TCPC_REG_CONTROL0_HOST_CUR_USB\n");
    // }

    // printf("%sTCPC_REG_CONTROL0_TX_START\n", (value & TCPC_REG_CONTROL0_TX_START) ? "" : "!");

	rv = tcpc_write(port, TCPC_REG_CONTROL0, reg);

	return rv;
}

int16_t fusb302_tcpm_init(int16_t port)
{
	int16_t reg;

	/* set default */
	state[port].cc_polarity = -1;

	/* set the voltage threshold for no connect detection (vOpen) */
	state[port].mdac_vnc = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_VNC_MV);
	/* set the voltage threshold for Rd vs Ra detection */
	state[port].mdac_rd = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_RD_THRESH_MV);

	/* all other variables assumed to default to 0 */

	/* Restore default settings */
	tcpc_write(port, TCPC_REG_RESET, TCPC_REG_RESET_SW_RESET);

	tcpc_read(port, TCPC_REG_DEVICE_ID, &reg);

	/* Turn on retries and set number of retries */
	tcpc_read(port, TCPC_REG_CONTROL3, &reg);
	reg |= TCPC_REG_CONTROL3_AUTO_RETRY;
	reg |= (PD_RETRY_COUNT & 0x3) << TCPC_REG_CONTROL3_N_RETRIES_POS;
	reg |= TCPC_REG_CONTROL3_SEND_HARDRESET;
	tcpc_write(port, TCPC_REG_CONTROL3, reg);

	/* Create interrupt masks */
	reg = 0x0;
	/* VBUS OK */
	reg &= ~TCPC_REG_MASK_VBUSOK;
    /* A change in host requested current level has occured */
	reg &= ~TCPC_REG_MASK_BC_LVL;
	/* collisions */
	reg &= ~TCPC_REG_MASK_COLLISION;
	/* misc alert */
	reg &= ~TCPC_REG_MASK_ALERT;
	/* packet received with correct CRC */
	reg &= ~TCPC_REG_MASK_CRC_CHK;
	tcpc_write(port, TCPC_REG_MASK, reg);

	reg = 0x0;
	/* when all pd message retries fail... */
	reg &= ~TCPC_REG_MASKA_RETRYFAIL;
	/* when fusb302 send a hard reset. */
	reg &= ~TCPC_REG_MASKA_HARDSENT;
	/* when fusb302 receives GoodCRC ack for a pd message */
	reg &= ~TCPC_REG_MASKA_TX_SUCCESS;
	/* when fusb302 receives a hard reset */
	reg &= ~TCPC_REG_MASKA_HARDRESET;
	tcpc_write(port, TCPC_REG_MASKA, reg);

	reg = 0xFF;
	/* when fusb302 sends GoodCRC to ack a pd message */
	reg &= ~TCPC_REG_MASKB_GCRCSENT;
	tcpc_write(port, TCPC_REG_MASKB, reg);

	/* Interrupt Enable */
	tcpc_read(port, TCPC_REG_CONTROL0, &reg);
	reg &= ~TCPC_REG_CONTROL0_INT_MASK;
	tcpc_write(port, TCPC_REG_CONTROL0, reg);

	state[port].control1 =
	    TCPC_REG_CONTROL1_RX_FLUSH | TCPC_REG_CONTROL1_ENSOP1DB |
	    TCPC_REG_CONTROL1_ENSOP2DB | TCPC_REG_CONTROL1_ENSOP1 | TCPC_REG_CONTROL1_ENSOP2;
		//   state[port].control1 = TCPC_REG_CONTROL1_RX_FLUSH | TCPC_REG_CONTROL1_ENSOP1DB | TCPC_REG_CONTROL1_ENSOP2DB;

	tcpc_write(port, TCPC_REG_CONTROL1, state[port].control1);

	fusb302_auto_goodcrc_enable(port, 0);

	/* Turn on the power! */
	/* TODO: Reduce power consumption */
	tcpc_write(port, TCPC_REG_POWER, TCPC_REG_POWER_PWR_ALL);

	return 0;
}

int16_t fusb302_tcpm_get_cc(int16_t port, int16_t *cc1, int16_t *cc2)
{
	if (state[port].pulling_up) {
		/* Source mode? */
		detect_cc_pin_source_manual(port, cc1, cc2);
	} else {
		/* Sink mode? */
		detect_cc_pin_sink(port, cc1, cc2);
	}

	return 0;
}

int16_t fusb302_tcpm_set_cc(int16_t port, int16_t pull)
{
	int16_t reg;

	/* NOTE: FUSB302 toggles a single pull-up between CC1 and CC2 */
	/* NOTE: FUSB302 Does not support Ra. */
	switch (pull) {
	case TYPEC_CC_RP:
		/* enable the pull-up we know to be necessary */
		tcpc_read(port, TCPC_REG_SWITCHES0, &reg);

		reg &= ~(TCPC_REG_SWITCHES0_CC2_PU_EN |
			 TCPC_REG_SWITCHES0_CC1_PU_EN |
			 TCPC_REG_SWITCHES0_CC1_PD_EN |
			 TCPC_REG_SWITCHES0_CC2_PD_EN |
			 TCPC_REG_SWITCHES0_VCONN_CC1 |
			 TCPC_REG_SWITCHES0_VCONN_CC2);

		reg |= TCPC_REG_SWITCHES0_CC1_PU_EN |
		    TCPC_REG_SWITCHES0_CC2_PU_EN;

		if (state[port].vconn_enabled)
			reg |= state[port].cc_polarity ?
			    TCPC_REG_SWITCHES0_VCONN_CC1 :
			    TCPC_REG_SWITCHES0_VCONN_CC2;
	uint8_t value = reg;

    // printf("%sTCPC_REG_SWITCHES0_CC2_PU_EN\n", (value & TCPC_REG_SWITCHES0_CC2_PU_EN) ? "" : "!");
    // printf("%sTCPC_REG_SWITCHES0_CC1_PU_EN\n", (value & TCPC_REG_SWITCHES0_CC1_PU_EN) ? "" : "!");
    // printf("%sTCPC_REG_SWITCHES0_VCONN_CC2\n", (value & TCPC_REG_SWITCHES0_VCONN_CC2) ? "" : "!");
    // printf("%sTCPC_REG_SWITCHES0_VCONN_CC1\n", (value & TCPC_REG_SWITCHES0_VCONN_CC1) ? "" : "!");
    // printf("%sTCPC_REG_SWITCHES0_MEAS_CC2\n", (value & TCPC_REG_SWITCHES0_MEAS_CC2) ? "" : "!");
    // printf("%sTCPC_REG_SWITCHES0_MEAS_CC1\n", (value & TCPC_REG_SWITCHES0_MEAS_CC1) ? "" : "!");
    // printf("%sTCPC_REG_SWITCHES0_CC2_PD_EN\n", (value & TCPC_REG_SWITCHES0_CC2_PD_EN) ? "" : "!");
    // printf("%sTCPC_REG_SWITCHES0_CC1_PD_EN\n", (value & TCPC_REG_SWITCHES0_CC1_PD_EN) ? "" : "!");

		tcpc_write(port, TCPC_REG_SWITCHES0, reg);

		state[port].pulling_up = 1;
		break;
	case TYPEC_CC_RD:
		/* Enable UFP Mode */

		/* turn off toggle */
		tcpc_read(port, TCPC_REG_CONTROL2, &reg);
		reg &= ~TCPC_REG_CONTROL2_TOGGLE;
		tcpc_write(port, TCPC_REG_CONTROL2, reg);

		/* enable pull-downs, disable pullups */
		tcpc_read(port, TCPC_REG_SWITCHES0, &reg);

		reg &= ~(TCPC_REG_SWITCHES0_CC2_PU_EN);
		reg &= ~(TCPC_REG_SWITCHES0_CC1_PU_EN);
		reg |= (TCPC_REG_SWITCHES0_CC1_PD_EN);
		reg |= (TCPC_REG_SWITCHES0_CC2_PD_EN);
		tcpc_write(port, TCPC_REG_SWITCHES0, reg);

		state[port].pulling_up = 0;
		break;
	case TYPEC_CC_OPEN:
		/* Disable toggling */
		tcpc_read(port, TCPC_REG_CONTROL2, &reg);
		reg &= ~TCPC_REG_CONTROL2_TOGGLE;
		tcpc_write(port, TCPC_REG_CONTROL2, reg);

		/* Ensure manual switches are opened */
		tcpc_read(port, TCPC_REG_SWITCHES0, &reg);
		reg &= ~TCPC_REG_SWITCHES0_CC1_PU_EN;
		reg &= ~TCPC_REG_SWITCHES0_CC2_PU_EN;
		reg &= ~TCPC_REG_SWITCHES0_CC1_PD_EN;
		reg &= ~TCPC_REG_SWITCHES0_CC2_PD_EN;
		tcpc_write(port, TCPC_REG_SWITCHES0, reg);

		state[port].pulling_up = 0;
		break;
	default:
		/* Unsupported... */
		return EC_ERROR_UNIMPLEMENTED;
	}

	return 0;
}

int16_t fusb302_tcpm_set_polarity(int16_t port, int16_t polarity)
{
	/* Port polarity : 0 => CC1 is CC line, 1 => CC2 is CC line */
	int16_t reg;

	tcpc_read(port, TCPC_REG_SWITCHES0, &reg);

	/* clear VCONN switch bits */
	reg &= ~TCPC_REG_SWITCHES0_VCONN_CC1;
	reg &= ~TCPC_REG_SWITCHES0_VCONN_CC2;

	if (state[port].vconn_enabled) {
		/* set VCONN switch to be non-CC line */
		if (polarity) {
			reg |= TCPC_REG_SWITCHES0_VCONN_CC1;
			reg &= ~TCPC_REG_SWITCHES0_CC1_PU_EN;
		} else {
			reg |= TCPC_REG_SWITCHES0_VCONN_CC2;
			reg &= ~TCPC_REG_SWITCHES0_CC2_PU_EN;
		}
	}

	/* clear meas_cc bits (RX line select) */
	reg &= ~TCPC_REG_SWITCHES0_MEAS_CC1;
	reg &= ~TCPC_REG_SWITCHES0_MEAS_CC2;

	/* set rx polarity */
	if (polarity)
		reg |= TCPC_REG_SWITCHES0_MEAS_CC2;
	else
		reg |= TCPC_REG_SWITCHES0_MEAS_CC1;

	tcpc_write(port, TCPC_REG_SWITCHES0, reg);

	tcpc_read(port, TCPC_REG_SWITCHES1, &reg);

	/* clear tx_cc bits */
	reg &= ~TCPC_REG_SWITCHES1_TXCC1_EN;
	reg &= ~TCPC_REG_SWITCHES1_TXCC2_EN;

	/* set tx polarity */
	if (polarity)
		reg |= TCPC_REG_SWITCHES1_TXCC2_EN;
	else
		reg |= TCPC_REG_SWITCHES1_TXCC1_EN;

	tcpc_write(port, TCPC_REG_SWITCHES1, reg);

	/* Save the polarity for later */
	state[port].cc_polarity = polarity;

	return 0;
}

int16_t fusb302_tcpm_set_msg_header(int16_t port, int16_t power_role, int16_t data_role)
{
	int16_t reg;

	tcpc_read(port, TCPC_REG_SWITCHES1, &reg);

	reg &= ~TCPC_REG_SWITCHES1_POWERROLE;
	reg &= ~TCPC_REG_SWITCHES1_DATAROLE;

	if (power_role)
		reg |= TCPC_REG_SWITCHES1_POWERROLE;
	if (data_role)
		reg |= TCPC_REG_SWITCHES1_DATAROLE;

	tcpc_write(port, TCPC_REG_SWITCHES1, reg);

	return 0;
}

int16_t fusb302_tcpm_set_rx_enable(int16_t port, int16_t enable)
{
	int16_t reg;

	state[port].rx_enable = enable;

	/* Get current switch state */
	tcpc_read(port, TCPC_REG_SWITCHES0, &reg);

	/* Clear CC1/CC2 measure bits */
	reg &= ~TCPC_REG_SWITCHES0_MEAS_CC1;
	reg &= ~TCPC_REG_SWITCHES0_MEAS_CC2;

	if (enable) {
		switch (state[port].cc_polarity) {
			/* if CC polarity hasnt been determined, can't enable */
		case -1:
			return EC_ERROR_UNKNOWN;
		case 0:
			reg |= TCPC_REG_SWITCHES0_MEAS_CC1;
			break;
		case 1:
			reg |= TCPC_REG_SWITCHES0_MEAS_CC2;
			break;
		default:
			/* "shouldn't get here" */
			return EC_ERROR_UNKNOWN;
		}
		tcpc_write(port, TCPC_REG_SWITCHES0, reg);

		/* Disable BC_LVL interrupt when enabling PD comm */
		if (!tcpc_read(port, TCPC_REG_MASK, &reg))
			tcpc_write(port, TCPC_REG_MASK,
				   reg | TCPC_REG_MASK_BC_LVL);

		/* flush rx fifo in case messages have been coming our way */
		fusb302_flush_rx_fifo(port);

	} else {
		tcpc_write(port, TCPC_REG_SWITCHES0, reg);

		/* Enable BC_LVL interrupt when disabling PD comm */
		if (!tcpc_read(port, TCPC_REG_MASK, &reg))
			tcpc_write(port, TCPC_REG_MASK,
				   reg & ~TCPC_REG_MASK_BC_LVL);
	}

	fusb302_auto_goodcrc_enable(port, enable);

	return 0;
}

/* Return true if our Rx FIFO is empty */
int16_t fusb302_rx_fifo_is_empty(int16_t port)
{
	int16_t reg, ret;

	ret = (!tcpc_read(port, TCPC_REG_STATUS1, &reg)) &&
	    (reg & TCPC_REG_STATUS1_RX_EMPTY);

	return ret;
}

int16_t fusb302_tcpm_get_message(int16_t port, uint32_t * payload, int16_t *head,
			     enum fusb302_rxfifo_tokens *sop)
{
	/*
	 * This is the buffer that will get the burst-read data
	 * from the fusb302.
	 *
	 * It's re-used in a couple different spots, the worst of which
	 * is the PD packet (not header) and CRC.
	 * maximum size necessary = 28 + 4 = 32
	 */
	uint8_t buf[32];
	int16_t rv, len;

	/* If our FIFO is empty then we have no packet */
	if (fusb302_rx_fifo_is_empty(port))
		return EC_ERROR_UNKNOWN;

	/* Read until we have a non-GoodCRC packet or an empty FIFO */
	do {
		buf[0] = TCPC_REG_FIFOS;

		/*
		 * PART 1 OF BURST READ: Write in register address.
		 * Issue a START, no STOP.
		 */
		rv = tcpc_xfer(port, buf, 1, 0, 0, I2C_XFER_START);

		/*
		 * PART 2 OF BURST READ: Read up to the header.
		 * Issue a repeated START, no STOP.
		 * only grab three bytes so we can get the header
		 * and determine how many more bytes we need to read.
		 * TODO: Check token to ensure valid packet.
		 */
		rv |= tcpc_xfer(port, 0, 0, buf, 3, I2C_XFER_START);

		/* Grab the header */
		*sop = buf[0] & fusb302_TKN_SOP_MASK;
		*head = (buf[1] & 0xFF);
		*head |= ((buf[2] << 8) & 0xFF00);

		/* figure out packet length, subtract header bytes */
		len = get_num_bytes(*head) - 2;

		/*
		 * PART 3 OF BURST READ: Read everything else.
		 * No START, but do issue a STOP at the end.
		 * add 4 to len to read CRC out
		 */
		rv |= tcpc_xfer(port, 0, 0, buf, len + 4, I2C_XFER_STOP);

	} while (!rv && PACKET_IS_GOOD_CRC(*head) &&
		 !fusb302_rx_fifo_is_empty(port));

	if (!rv) {
		/* Discard GoodCRC packets */
		if (PACKET_IS_GOOD_CRC(*head))
			rv = EC_ERROR_UNKNOWN;
		else
			memcpy(payload, buf, len);
	}

	/*
	 * If our FIFO is non-empty then we may have a packet, we may get
	 * fewer interrupts than packets due to interrupt latency.
	 */
	//if (!fusb302_rx_fifo_is_empty(port))
	//  task_set_event(PD_PORT_TO_TASK_ID(port), PD_EVENT_RX, 0);

	return rv;
}

int16_t fusb302_tcpm_transmit(int16_t port, enum tcpm_transmit_type type,
			  uint16_t header, const uint32_t * data)
{
	/*
	 * this is the buffer that will be burst-written into the fusb302
	 * maximum size necessary =
	 * 1: FIFO register address
	 * 4: SOP* tokens
	 * 1: Token that signifies "next X bytes are not tokens"
	 * 30: 2 for header and up to 7*4 = 28 for rest of message
	 * 1: "Insert CRC" Token
	 * 1: EOP Token
	 * 1: "Turn transmitter off" token
	 * 1: "Star Transmission" Command
	 * -
	 * 40: 40 bytes worst-case
	 */
	uint8_t buf[40];
	int16_t buf_pos = 0;

	int16_t reg;

	/* Flush the TXFIFO */
	fusb302_flush_tx_fifo(port);

	header |= state[port].msgid++ << 9;
	state[port].msgid &= 0x7;

	switch (type) {
	case TCPC_TX_SOP:

		/* put register address first for of burst tcpc write */
		buf[buf_pos++] = TCPC_REG_FIFOS;

		/* Write the SOP Ordered Set into TX FIFO */
		buf[buf_pos++] = fusb302_TKN_SYNC1;
		buf[buf_pos++] = fusb302_TKN_SYNC1;
		buf[buf_pos++] = fusb302_TKN_SYNC1;
		buf[buf_pos++] = fusb302_TKN_SYNC2;

		fusb302_send_message(port, header, data, buf, buf_pos);
		// wait for the GoodCRC to come back before we let the rest
		// of the code do stuff like change polarity and miss it
		platform_usleep(1200);
		return 0;
	case TCPC_TX_SOP_PRIME:

		/* put register address first for of burst tcpc write */
		buf[buf_pos++] = TCPC_REG_FIFOS;

		/* Write the SOP Ordered Set into TX FIFO */
		buf[buf_pos++] = fusb302_TKN_SYNC1;
		buf[buf_pos++] = fusb302_TKN_SYNC1;
		buf[buf_pos++] = fusb302_TKN_SYNC3;
		buf[buf_pos++] = fusb302_TKN_SYNC3;

		fusb302_send_message(port, header, data, buf, buf_pos);
		// wait for the GoodCRC to come back before we let the rest
		// of the code do stuff like change polarity and miss it
		platform_usleep(1200);
		return 0;
	case TCPC_TX_SOP_PRIME_PRIME:

		/* put register address first for of burst tcpc write */
		buf[buf_pos++] = TCPC_REG_FIFOS;

		/* Write the SOP Ordered Set into TX FIFO */
		buf[buf_pos++] = fusb302_TKN_SYNC1;
		buf[buf_pos++] = fusb302_TKN_SYNC3;
		buf[buf_pos++] = fusb302_TKN_SYNC1;
		buf[buf_pos++] = fusb302_TKN_SYNC3;

		fusb302_send_message(port, header, data, buf, buf_pos);
		// wait for the GoodCRC to come back before we let the rest
		// of the code do stuff like change polarity and miss it
		platform_usleep(1200);
		return 0;
	case TCPC_TX_SOP_DEBUG_PRIME:

		/* put register address first for of burst tcpc write */
		buf[buf_pos++] = TCPC_REG_FIFOS;

		/* Write the SOP Ordered Set into TX FIFO */
		buf[buf_pos++] = fusb302_TKN_SYNC1;
		buf[buf_pos++] = fusb302_TKN_RST2;
		buf[buf_pos++] = fusb302_TKN_RST2;
		buf[buf_pos++] = fusb302_TKN_SYNC3;

		fusb302_send_message(port, header, data, buf, buf_pos);
		// wait for the GoodCRC to come back before we let the rest
		// of the code do stuff like change polarity and miss it
		platform_usleep(1200);
		return 0;
	case TCPC_TX_SOP_DEBUG_PRIME_PRIME:

		/* put register address first for of burst tcpc write */
		buf[buf_pos++] = TCPC_REG_FIFOS;

		/* Write the SOP Ordered Set into TX FIFO */
		buf[buf_pos++] = fusb302_TKN_SYNC1;
		buf[buf_pos++] = fusb302_TKN_RST2;
		buf[buf_pos++] = fusb302_TKN_SYNC3;
		buf[buf_pos++] = fusb302_TKN_SYNC2;

		fusb302_send_message(port, header, data, buf, buf_pos);
		// wait for the GoodCRC to come back before we let the rest
		// of the code do stuff like change polarity and miss it
		platform_usleep(1200);
		return 0;
	case TCPC_TX_HARD_RESET:
		/* Simply hit the SEND_HARD_RESET bit */
		tcpc_read(port, TCPC_REG_CONTROL3, &reg);
		reg |= TCPC_REG_CONTROL3_SEND_HARDRESET;
		tcpc_write(port, TCPC_REG_CONTROL3, reg);

		break;
	case TCPC_TX_BIST_MODE_2:
		/* Hit the BIST_MODE2 bit and start TX */
		tcpc_read(port, TCPC_REG_CONTROL1, &reg);
		reg |= TCPC_REG_CONTROL1_BIST_MODE2;
		tcpc_write(port, TCPC_REG_CONTROL1, reg);

		tcpc_read(port, TCPC_REG_CONTROL0, &reg);
		reg |= TCPC_REG_CONTROL0_TX_START;
		reg |= TCPC_REG_CONTROL0_HOST_CUR_USB;
		tcpc_write(port, TCPC_REG_CONTROL0, reg);

		//task_wait_event(PD_T_BIST_TRANSMIT);

		/* Clear BIST mode bit, TX_START is self-clearing */
		tcpc_read(port, TCPC_REG_CONTROL1, &reg);
		reg &= ~TCPC_REG_CONTROL1_BIST_MODE2;
		tcpc_write(port, TCPC_REG_CONTROL1, reg);

		break;
	default:
		return EC_ERROR_UNIMPLEMENTED;
	}

	return 0;
}

int16_t fusb302_tcpm_get_vbus_level(int16_t port)
{
	int16_t reg;

	/* Read status register */
	tcpc_read(port, TCPC_REG_STATUS0, &reg);

	return (reg & TCPC_REG_STATUS0_VBUSOK) ? 1 : 0;
}

void fusb302_get_irq(int16_t port, int16_t *interrupt, int16_t *interrupta, int16_t *interruptb)
{
	/* reading interrupt registers clears them */

	tcpc_read(port, TCPC_REG_INTERRUPT, interrupt);
	tcpc_read(port, TCPC_REG_INTERRUPTA, interrupta);
	tcpc_read(port, TCPC_REG_INTERRUPTB, interruptb);

#if 0
	/*
	 * Ignore BC_LVL changes when transmitting / receiving PD,
	 * since CC level will constantly change.
	 */
	if (state[port].rx_enable)
		interrupt &= ~TCPC_REG_INTERRUPT_BC_LVL;

	if (interrupt & TCPC_REG_INTERRUPT_BC_LVL) {
		/* CC Status change */
		//task_set_event(PD_PORT_TO_TASK_ID(port), PD_EVENT_CC, 0);
	}

	if (interrupt & TCPC_REG_INTERRUPT_COLLISION) {
		/* packet sending collided */
		pd_transmit_complete(port, TCPC_TX_COMPLETE_FAILED);
	}

	/* GoodCRC was received, our FIFO is now non-empty */
	if (interrupta & TCPC_REG_INTERRUPTA_TX_SUCCESS) {
		//task_set_event(PD_PORT_TO_TASK_ID(port),
		//    PD_EVENT_RX, 0);

		pd_transmit_complete(port, TCPC_TX_COMPLETE_SUCCESS);
	}

	if (interrupta & TCPC_REG_INTERRUPTA_RETRYFAIL) {
		/* all retries have failed to get a GoodCRC */
		pd_transmit_complete(port, TCPC_TX_COMPLETE_FAILED);
	}

	if (interrupta & TCPC_REG_INTERRUPTA_HARDSENT) {
		/* hard reset has been sent */

		/* bring FUSB302 out of reset */
		fusb302_pd_reset(port);

		pd_transmit_complete(port, TCPC_TX_COMPLETE_SUCCESS);
	}

	if (interrupta & TCPC_REG_INTERRUPTA_HARDRESET) {
		/* hard reset has been received */

		/* bring FUSB302 out of reset */
		fusb302_pd_reset(port);

		pd_execute_hard_reset(port);

		//task_wake(PD_PORT_TO_TASK_ID(port));
	}

	if (interruptb & TCPC_REG_INTERRUPTB_GCRCSENT) {
		/* Packet received and GoodCRC sent */
		/* (this interrupt fires after the GoodCRC finishes) */
		if (state[port].rx_enable) {
			//task_set_event(PD_PORT_TO_TASK_ID(port),
			//    PD_EVENT_RX, 0);
		} else {
			/* flush rx fifo if rx isn't enabled */
			fusb302_flush_rx_fifo(port);
		}
	}
#endif
}

int16_t fusb302_tcpm_set_vconn(int16_t port, int16_t enable)
{
	/*
	 * FUSB302 does not have dedicated VCONN Enable switch.
	 * We'll get through this by disabling both of the
	 * VCONN - CC* switches to disable, and enabling the
	 * saved polarity when enabling.
	 * Therefore at startup, set_polarity should be called first,
	 * or else live with the default put into init.
	 */
	int16_t reg;

	/* save enable state for later use */
	state[port].vconn_enabled = enable;

	if (enable) {
		/* set to saved polarity */
		fusb302_tcpm_set_polarity(port, state[port].cc_polarity);
	} else {

		tcpc_read(port, TCPC_REG_SWITCHES0, &reg);

		/* clear VCONN switch bits */
		reg &= ~TCPC_REG_SWITCHES0_VCONN_CC1;
		reg &= ~TCPC_REG_SWITCHES0_VCONN_CC2;

		tcpc_write(port, TCPC_REG_SWITCHES0, reg);
	}

	return 0;
}

#if 0
/* For BIST receiving */
void tcpm_set_bist_test_data(int16_t port)
{
	int16_t reg;

	/* Read control3 register */
	tcpc_read(port, TCPC_REG_CONTROL3, &reg);

	/* Set the BIST_TMODE bit (Clears on Hard Reset) */
	reg |= TCPC_REG_CONTROL3_BIST_TMODE;

	/* Write the updated value */
	tcpc_write(port, TCPC_REG_CONTROL3, reg);
}
#endif
