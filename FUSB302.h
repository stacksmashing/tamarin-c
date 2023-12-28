/*
  FUSB302.h - Library for interacting with the FUSB302B chip.
  Copyright 2010 The Chromium OS Authors
  Copyright 2017 Jason Cerundolo
  Modified in 2023 by Thomas 'stacksmashing' Roth
*/

#ifndef fusb302_H
#define fusb302_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usb_pd_tcpm.h"

/* Chip Device ID - 302A or 302B */
#define fusb302_DEVID_302A 0x08
#define fusb302_DEVID_302B 0x09

/* I2C slave address varies by part number */
/* FUSB302BUCX / FUSB302BMPX */
#define fusb302_I2C_SLAVE_ADDR 0x22 // 7-bit address for Arduino
/* FUSB302B01MPX */
#define fusb302_I2C_SLAVE_ADDR_B01 0x23
/* FUSB302B10MPX */
#define fusb302_I2C_SLAVE_ADDR_B10 0x24
/* FUSB302B11MPX */
#define fusb302_I2C_SLAVE_ADDR_B11 0x25

/* Default retry count for transmitting */
#define PD_RETRY_COUNT      3

/* Time to wait for TCPC to complete transmit */
#define PD_T_TCPC_TX_TIMEOUT  (100*MSEC_US)

#define TCPC_REG_DEVICE_ID  0x01

#define TCPC_REG_SWITCHES0  0x02
#define TCPC_REG_SWITCHES0_CC2_PU_EN    (1<<7)
#define TCPC_REG_SWITCHES0_CC1_PU_EN    (1<<6)
#define TCPC_REG_SWITCHES0_VCONN_CC2    (1<<5)
#define TCPC_REG_SWITCHES0_VCONN_CC1    (1<<4)
#define TCPC_REG_SWITCHES0_MEAS_CC2 (1<<3)
#define TCPC_REG_SWITCHES0_MEAS_CC1 (1<<2)
#define TCPC_REG_SWITCHES0_CC2_PD_EN    (1<<1)
#define TCPC_REG_SWITCHES0_CC1_PD_EN    (1<<0)

#define TCPC_REG_SWITCHES1  0x03
#define TCPC_REG_SWITCHES1_POWERROLE    (1<<7)
#define TCPC_REG_SWITCHES1_SPECREV1 (1<<6)
#define TCPC_REG_SWITCHES1_SPECREV0 (1<<5)
#define TCPC_REG_SWITCHES1_DATAROLE (1<<4)
#define TCPC_REG_SWITCHES1_AUTO_GCRC    (1<<2)
#define TCPC_REG_SWITCHES1_TXCC2_EN (1<<1)
#define TCPC_REG_SWITCHES1_TXCC1_EN (1<<0)

#define TCPC_REG_MEASURE    0x04
#define TCPC_REG_MEASURE_VBUS       (1<<6)
#define TCPC_REG_MEASURE_MDAC_MV(mv)    (((mv)/42) & 0x3f)

#define TCPC_REG_CONTROL0   0x06
#define TCPC_REG_CONTROL0_TX_FLUSH  (1<<6)
#define TCPC_REG_CONTROL0_INT_MASK  (1<<5)
#define TCPC_REG_CONTROL0_HOST_CUR_MASK (3<<2)
#define TCPC_REG_CONTROL0_HOST_CUR_3A0  (3<<2)
#define TCPC_REG_CONTROL0_HOST_CUR_1A5  (2<<2)
#define TCPC_REG_CONTROL0_HOST_CUR_USB  (1<<2)
#define TCPC_REG_CONTROL0_TX_START  (1<<0)

#define TCPC_REG_CONTROL1   0x07
#define TCPC_REG_CONTROL1_ENSOP2DB  (1<<6)
#define TCPC_REG_CONTROL1_ENSOP1DB  (1<<5)
#define TCPC_REG_CONTROL1_BIST_MODE2    (1<<4)
#define TCPC_REG_CONTROL1_RX_FLUSH  (1<<2)
#define TCPC_REG_CONTROL1_ENSOP2    (1<<1)
#define TCPC_REG_CONTROL1_ENSOP1    (1<<0)

#define TCPC_REG_CONTROL2   0x08
/* two-bit field, valid values below */
#define TCPC_REG_CONTROL2_MODE      (1<<1)
#define TCPC_REG_CONTROL2_MODE_DFP  (0x3)
#define TCPC_REG_CONTROL2_MODE_UFP  (0x2)
#define TCPC_REG_CONTROL2_MODE_DRP  (0x1)
#define TCPC_REG_CONTROL2_MODE_POS  (1)
#define TCPC_REG_CONTROL2_TOGGLE    (1<<0)

#define TCPC_REG_CONTROL3   0x09
#define TCPC_REG_CONTROL3_SEND_HARDRESET    (1<<6)
#define TCPC_REG_CONTROL3_BIST_TMODE        (1<<5) /* 302B Only */
#define TCPC_REG_CONTROL3_AUTO_HARDRESET    (1<<4)
#define TCPC_REG_CONTROL3_AUTO_SOFTRESET    (1<<3)
/* two-bit field */
#define TCPC_REG_CONTROL3_N_RETRIES     (1<<1)
#define TCPC_REG_CONTROL3_N_RETRIES_POS     (1)
#define TCPC_REG_CONTROL3_N_RETRIES_SIZE    (2)
#define TCPC_REG_CONTROL3_AUTO_RETRY        (1<<0)

#define TCPC_REG_MASK       0x0A
#define TCPC_REG_MASK_VBUSOK        (1<<7)
#define TCPC_REG_MASK_ACTIVITY      (1<<6)
#define TCPC_REG_MASK_COMP_CHNG     (1<<5)
#define TCPC_REG_MASK_CRC_CHK       (1<<4)
#define TCPC_REG_MASK_ALERT     (1<<3)
#define TCPC_REG_MASK_WAKE      (1<<2)
#define TCPC_REG_MASK_COLLISION     (1<<1)
#define TCPC_REG_MASK_BC_LVL        (1<<0)

#define TCPC_REG_POWER      0x0B
#define TCPC_REG_POWER_PWR      (1<<0)  /* four-bit field */
#define TCPC_REG_POWER_PWR_LOW      0x1 /* Bandgap + Wake circuitry */
#define TCPC_REG_POWER_PWR_MEDIUM   0x3 /* LOW + Receiver + Current refs */
#define TCPC_REG_POWER_PWR_HIGH     0x7 /* MEDIUM + Measure block */
#define TCPC_REG_POWER_PWR_ALL      0xF /* HIGH + Internal Oscillator */

#define TCPC_REG_RESET      0x0C
#define TCPC_REG_RESET_PD_RESET     (1<<1)
#define TCPC_REG_RESET_SW_RESET     (1<<0)

#define TCPC_REG_MASKA      0x0E
#define TCPC_REG_MASKA_OCP_TEMP     (1<<7)
#define TCPC_REG_MASKA_TOGDONE      (1<<6)
#define TCPC_REG_MASKA_SOFTFAIL     (1<<5)
#define TCPC_REG_MASKA_RETRYFAIL    (1<<4)
#define TCPC_REG_MASKA_HARDSENT     (1<<3)
#define TCPC_REG_MASKA_TX_SUCCESS   (1<<2)
#define TCPC_REG_MASKA_SOFTRESET    (1<<1)
#define TCPC_REG_MASKA_HARDRESET    (1<<0)

#define TCPC_REG_MASKB      0x0F
#define TCPC_REG_MASKB_GCRCSENT     (1<<0)

#define TCPC_REG_STATUS0A   0x3C
#define TCPC_REG_STATUS0A_SOFTFAIL  (1<<5)
#define TCPC_REG_STATUS0A_RETRYFAIL (1<<4)
#define TCPC_REG_STATUS0A_POWER     (1<<2) /* two-bit field */
#define TCPC_REG_STATUS0A_RX_SOFT_RESET (1<<1)
#define TCPC_REG_STATUS0A_RX_HARD_RESET (1<<0)

#define TCPC_REG_STATUS1A   0x3D
/* three-bit field, valid values below */
#define TCPC_REG_STATUS1A_TOGSS     (1<<3)
#define TCPC_REG_STATUS1A_TOGSS_RUNNING     0x0
#define TCPC_REG_STATUS1A_TOGSS_SRC1        0x1
#define TCPC_REG_STATUS1A_TOGSS_SRC2        0x2
#define TCPC_REG_STATUS1A_TOGSS_SNK1        0x5
#define TCPC_REG_STATUS1A_TOGSS_SNK2        0x6
#define TCPC_REG_STATUS1A_TOGSS_AA      0x7
#define TCPC_REG_STATUS1A_TOGSS_POS     (3)
#define TCPC_REG_STATUS1A_TOGSS_MASK        (0x7)

#define TCPC_REG_STATUS1A_RXSOP2DB  (1<<2)
#define TCPC_REG_STATUS1A_RXSOP1DB  (1<<1)
#define TCPC_REG_STATUS1A_RXSOP     (1<<0)

#define TCPC_REG_INTERRUPTA 0x3E
#define TCPC_REG_INTERRUPTA_OCP_TEMP    (1<<7)
#define TCPC_REG_INTERRUPTA_TOGDONE (1<<6)
#define TCPC_REG_INTERRUPTA_SOFTFAIL    (1<<5)
#define TCPC_REG_INTERRUPTA_RETRYFAIL   (1<<4)
#define TCPC_REG_INTERRUPTA_HARDSENT    (1<<3)
#define TCPC_REG_INTERRUPTA_TX_SUCCESS  (1<<2)
#define TCPC_REG_INTERRUPTA_SOFTRESET   (1<<1)
#define TCPC_REG_INTERRUPTA_HARDRESET   (1<<0)

#define TCPC_REG_INTERRUPTB 0x3F
#define TCPC_REG_INTERRUPTB_GCRCSENT        (1<<0)

#define TCPC_REG_STATUS0    0x40
#define TCPC_REG_STATUS0_VBUSOK     (1<<7)
#define TCPC_REG_STATUS0_ACTIVITY   (1<<6)
#define TCPC_REG_STATUS0_COMP       (1<<5)
#define TCPC_REG_STATUS0_CRC_CHK    (1<<4)
#define TCPC_REG_STATUS0_ALERT      (1<<3)
#define TCPC_REG_STATUS0_WAKE       (1<<2)
#define TCPC_REG_STATUS0_BC_LVL1    (1<<1) /* two-bit field */
#define TCPC_REG_STATUS0_BC_LVL0    (1<<0) /* two-bit field */

#define TCPC_REG_STATUS1    0x41
#define TCPC_REG_STATUS1_RXSOP2     (1<<7)
#define TCPC_REG_STATUS1_RXSOP1     (1<<6)
#define TCPC_REG_STATUS1_RX_EMPTY   (1<<5)
#define TCPC_REG_STATUS1_RX_FULL    (1<<4)
#define TCPC_REG_STATUS1_TX_EMPTY   (1<<3)
#define TCPC_REG_STATUS1_TX_FULL    (1<<2)

#define TCPC_REG_INTERRUPT  0x42
#define TCPC_REG_INTERRUPT_VBUSOK   (1<<7)
#define TCPC_REG_INTERRUPT_ACTIVITY (1<<6)
#define TCPC_REG_INTERRUPT_COMP_CHNG    (1<<5)
#define TCPC_REG_INTERRUPT_CRC_CHK  (1<<4)
#define TCPC_REG_INTERRUPT_ALERT    (1<<3)
#define TCPC_REG_INTERRUPT_WAKE     (1<<2)
#define TCPC_REG_INTERRUPT_COLLISION    (1<<1)
#define TCPC_REG_INTERRUPT_BC_LVL   (1<<0)

#define TCPC_REG_FIFOS      0x43

/* Tokens defined for the FUSB302 TX FIFO */
enum fusb302_txfifo_tokens {
    fusb302_TKN_TXON = 0xA1,
    fusb302_TKN_SYNC1 = 0x12,
    fusb302_TKN_SYNC2 = 0x13,
    fusb302_TKN_SYNC3 = 0x1B,
    fusb302_TKN_RST1 = 0x15,
    fusb302_TKN_RST2 = 0x16,
    fusb302_TKN_PACKSYM = 0x80,
    fusb302_TKN_JAMCRC = 0xFF,
    fusb302_TKN_EOP = 0x14,
    fusb302_TKN_TXOFF = 0xFE,
};

enum fusb302_rxfifo_tokens {
    fusb302_TKN_SOP = 0xE0,
    fusb302_TKN_SOP1 = 0xC0,
    fusb302_TKN_SOP2 = 0xA0,
    fusb302_TKN_SOP1DB = 0x80,
    fusb302_TKN_SOP2DB = 0x60,
    fusb302_TKN_SOP_MASK = 0xE0,
};

extern const struct tcpm_drv fusb302_tcpm_drv;

// Common methods for TCPM implementations
int16_t fusb302_tcpm_init(int16_t port);
void fusb302_pd_reset(int16_t port);
void fusb302_flush_rx_fifo(int16_t port);
void fusb302_flush_tx_fifo(int16_t port);
void fusb302_auto_goodcrc_enable(int16_t port, int16_t enable);
int16_t fusb302_tcpm_get_cc(int16_t port, int16_t *cc1, int16_t *cc2);
int16_t fusb302_tcpm_set_cc(int16_t port, int16_t pull);
int16_t fusb302_tcpm_set_polarity(int16_t port, int16_t polarity);
int16_t fusb302_tcpm_set_vconn(int16_t port, int16_t enable);
int16_t fusb302_tcpm_set_msg_header(int16_t port, int16_t power_role, int16_t data_role);
int16_t fusb302_tcpm_set_rx_enable(int16_t port, int16_t enable);
int16_t fusb302_tcpm_get_message(int16_t port, uint32_t *payload, int16_t *head, enum fusb302_rxfifo_tokens *sop);
int16_t fusb302_tcpm_transmit(int16_t port, enum tcpm_transmit_type type, uint16_t header, const uint32_t *data);
int16_t fusb302_tcpm_get_vbus_level(int16_t port);
int16_t fusb302_tcpm_select_rp_value(int16_t port, int16_t rp);
void fusb302_get_irq(int16_t port, int16_t *irq, int16_t *irqa, int16_t *irqb);
int16_t fusb302_rx_fifo_is_empty(int16_t port);
int get_polarity();

void fusb302_setup_mdac();
void fusb302_setup_toggle();
void fusb302_enable_toggle();
void fusb302_disable_toggle();
int16_t fusb302_measure_cc_pin_source(int16_t port, int16_t cc_to_measure);

#ifdef __cplusplus
}
#endif

#endif /* fusb302_H */
