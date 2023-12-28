/*  
    Tamarin-C - Tool for exploring USB-C on iPhones & macs
    Copyright 2023 Thomas 'stacksmashing' Roth - code@stacksmashing.net

    Heavily based on previous work by the AsahiLinux project, Marc Zyngier, etc.
    Check out: vdmtool and macvdmtool

    Released under GPLv3. See LICENSE for details.
*/

#pragma once

#include "tamarin_hw.h"
#include "usb_pd_tcpm.h"
#include "tcpm_driver.h"


enum CC_POLARITY {
    CC_POLARITY_CC1 = 0,
    CC_POLARITY_CC2
};

enum NEW_USB_STATE {
    USB_STATE_NONE,
    USB_STATE_FUSB_NOT_FOUND,
    // Wait for a comparator change which indciates a plug event
    USB_STATE_WAITING_FOR_COMP_CHNG,
    // After we send our source capabilities we wait for the "TX Success"
    // response
    // USB_STATE_WAIT_SOURCE_CAP_TX_SUCCESS,
    USB_STATE_WAIT_REQUEST,
    USB_STATE_IDLE
};

typedef struct tamarin_usb_pd {
    bool interrupt_pending;
    enum NEW_USB_STATE state;
    i2c_inst_t *i2c;
    uint32_t pin_int;
    uint32_t pin_scl;
    uint32_t pin_sda;

    void (*initialized_callback)(struct tamarin_usb_pd * usb_pd);

    // Logging configuration
    bool log_interrupts;
    bool log_messages;


    /* The iPhone 14 when connected by USB-C -> Lightning cable
       reguarly sends power requests. This needs to be true to
       allow iPhone 14 to keep charging, but might cause other issues.
       We're probably doing something wrong with the PD setup, so hopefully
       this will eventually be fixed. */
    bool handle_messages_in_idle;
} tamarin_usb_pd;



void vbus_on();
void vbus_off();

bool usb_pd_init(tamarin_usb_pd *usb_pd, uint32_t pin_scl, uint32_t pin_sda, uint32_t pin_int);
void usb_pd_enable_interrupt(tamarin_usb_pd *usb_pd);
void usb_pd_disable_interrupt(tamarin_usb_pd *usb_pd);
void usb_pd_handle_interrupt(tamarin_usb_pd *usb_pd);
bool usb_pd_get_interrupt(tamarin_usb_pd *usb_pd, int16_t *irq, int16_t *irqa, int16_t *irqb);
void usb_pd_parse_interrupt(int16_t irq, int16_t irqa, int16_t irqb);
