/*  
    Tamarin-C - Tool for exploring USB-C on iPhones & macs
    Copyright 2023 Thomas 'stacksmashing' Roth - code@stacksmashing.net

    Heavily based on previous work by the AsahiLinux project, Marc Zyngier, etc.
    Check out: vdmtool and macvdmtool

    Released under GPLv3. See LICENSE for details.
*/

#include "usb_pd.h"

#include "hardware/gpio.h"

#include <stdio.h>
#include <stdarg.h>

int16_t POWER = TYPEC_RP_3A0;

static tamarin_usb_pd *interrupt_usb_instance;

void mprintf(tamarin_usb_pd *usb_pd, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vuprintf(format, args);
    va_end(args);
}

void vbus_on(tamarin_usb_pd *usb_pd)
{
    mprintf(usb_pd, "VBUS ON\r\n");
    gpio_put(PIN_VBUS_EN, 1);
}

void vbus_off(tamarin_usb_pd *usb_pd)
{
    mprintf(usb_pd, "VBUS OFF\r\n");
    gpio_put(PIN_VBUS_EN, 0);
}

void usb_pd_parse_interrupt(int16_t irq, int16_t irqa, int16_t irqb)
{
    uprintf("Interrupt parsing of: %x\r\n", (unsigned int)irq);
    bool vbusok = irq & TCPC_REG_INTERRUPT_VBUSOK;
    bool activity = irq & TCPC_REG_INTERRUPT_ACTIVITY;
    bool comp_chng = irq & TCPC_REG_INTERRUPT_COMP_CHNG;
    bool crc_chk = irq & TCPC_REG_INTERRUPT_CRC_CHK;
    bool alert = irq & TCPC_REG_INTERRUPT_ALERT;
    bool wake = irq & TCPC_REG_INTERRUPT_WAKE;
    bool collision = irq & TCPC_REG_INTERRUPT_COLLISION;
    bool bc_lvl = irq & TCPC_REG_INTERRUPT_BC_LVL;
    uprintf("\tIRQ %s %s %s %s %s %s %s %s\r\n",
           vbusok ? "VBUSOK" : "",
           activity ? "ACTIVITY" : "",
           comp_chng ? "COMP_CHNG" : "",
           crc_chk ? "CRC_CHK" : "",
           alert ? "ALERT" : "",
           wake ? "WAKE" : "",
           collision ? "COLLISION" : "",
           bc_lvl ? "BC_LVL" : "");

    bool ocp_temp = irqa & TCPC_REG_INTERRUPTA_OCP_TEMP;
    bool togdone = irqa & TCPC_REG_INTERRUPTA_TOGDONE;
    bool softfail = irqa & TCPC_REG_INTERRUPTA_SOFTFAIL;
    bool retryfail = irqa & TCPC_REG_INTERRUPTA_RETRYFAIL;
    bool hardsent = irqa & TCPC_REG_INTERRUPTA_HARDSENT;
    bool tx_success = irqa & TCPC_REG_INTERRUPTA_TX_SUCCESS;
    bool softreset = irqa & TCPC_REG_INTERRUPTA_SOFTRESET;
    bool hardreset = irqa & TCPC_REG_INTERRUPTA_HARDRESET;

    uprintf("\tIRQA %s %s %s %s %s %s %s %s\r\n",
           ocp_temp ? "OCP_TEMP" : "",
           togdone ? "TOGDONE" : "",
           softfail ? "SOFTFAIL" : "",
           retryfail ? "RETRYFAIL" : "",
           hardsent ? "HARDSENT" : "",
           tx_success ? "TX_SUCCESS" : "",
           softreset ? "SOFTRESET" : "",
           hardreset ? "HARDRESET" : "");

    bool gcrcsent = irqb & TCPC_REG_INTERRUPTB_GCRCSENT;
    uprintf("\tIRQB %s\r\n", gcrcsent ? "GCRCSENT" : "");
}

bool usb_pd_get_interrupt(tamarin_usb_pd *usb_pd, int16_t *irq, int16_t *irqa, int16_t *irqb)
{
    if (gpio_get(usb_pd->pin_int))
    {
        return 0;
    }
    fusb302_get_irq(0, irq, irqa, irqb);

    if (usb_pd->log_interrupts)
    {
        uprintf("IRQ %x %x %x (Time: %d)\r\n", *irq, *irqa, *irqb, to_ms_since_boot(get_absolute_time()));
        usb_pd_parse_interrupt(*irq, *irqa, *irqb);
    }

    return 1;
}

static uint32_t build_fixed_pdo()
{
    // USB PD R2 0 V1.3 Table 6-6
    // Bit(s) Description
    // B31…30 Fixed supply
    // B29 Dual-Role Power
    // B28 USB Suspend Supported
    // B27 Unconstrained Power
    // B26 USB Communications Capable
    // B25 Dual-Role Data
    // B24…22 Reserved – Shall be set to zero.
    // B21…20 Peak Current
    // B19…10 Voltage in 50mV units
    // B9…0 Maximum Current in 10mA units
    uint32_t r = (0 << 31) | (1 << 26) | // usb comms capable
                 (100 << 10) |           // 5 volt = 100 * 50mV
                 (50 << 0);              // 500mA = 150 * 10mA
    return r;
}

static void dump_msg(tamarin_usb_pd *usb_pd, enum fusb302_rxfifo_tokens sop, int hdr, uint32_t *msg)
{
    if (!usb_pd->log_messages)
        return;
    int len = PD_HEADER_CNT(hdr);
    switch (sop)
    {
    case fusb302_TKN_SOP:
        uprintf("RX SOP (");
        break;
    case fusb302_TKN_SOP1:
        uprintf("RX SOP' (");
        break;
    case fusb302_TKN_SOP2:
        uprintf("RX SOP\" (");
        break;
    case fusb302_TKN_SOP1DB:
        uprintf("RX SOP'DEBUG (");
        break;
    case fusb302_TKN_SOP2DB:
        uprintf("RX SOP\"DEBUG (");
        break;
    default:
        uprintf("RX ? (");
        break;
    }

    uprintf("%d) [ 0x%X]", len, hdr);
    for (int i = 0; i < PD_HEADER_CNT(hdr); i++)
    {
        uprintf(" %08X", msg[i]);
    }
    uprintf("\r\n");
}

static void send_source_cap(tamarin_usb_pd *usb_pd)
{
    int16_t hdr = PD_HEADER(
        PD_DATA_SOURCE_CAP, // type
        1,                  // power role (1 = source)
        1,                  // data role (1 = dfp)
        0,                  // message id
        1,                  // number of objects
        PD_REV20,           // specification of revision
        0);                 // reserved lol?
                            // uint32_t cap = 1UL << 31; /* Variable non-battery PS, 0V, 0mA */
    uint32_t cap = build_fixed_pdo();

    fusb302_tcpm_transmit(0, TCPC_TX_SOP, hdr, &cap);
    mprintf(usb_pd, ">SOURCE_CAP\r\n");
}

static void handle_vdm(tamarin_usb_pd *usb_pd, enum fusb302_rxfifo_tokens sop,
                       int16_t hdr, uint32_t *msg)
{
    switch (*msg)
    {
    case 0xff008001: // Structured VDM: DISCOVER IDENTITY
        mprintf(usb_pd, "<VDM DISCOVER_IDENTITY\r\n");
        // handle_discover_identity();
        break;
    default:
        mprintf(usb_pd, "<VDM ");
        dump_msg(usb_pd, sop, hdr, msg);
        break;
    }
}
void handle_power_request(tamarin_usb_pd *usb_pd, uint32_t req)
{
    // int16_t hdr = PD_HEADER(
    // PD_DATA_SOURCE_CAP, // type
    // 1, // power role (1 = source)
    // 1, // data role (1 = dfp)
    // 0, // message id
    // 1, // number of objects
    // PD_REV20, // specification of revision
    // 0); // reserved lol?
    int hdr = PD_HEADER(
        PD_CTRL_ACCEPT,
        1,        // power role
        1,        // data role
        0,        // mesage id
        0,        // number of objects
        PD_REV20, // spec
        0);

    fusb302_tcpm_transmit(0, TCPC_TX_SOP, hdr, NULL);
    mprintf(usb_pd, ">ACCEPT\r\n");
    // vbus_on();
    sleep_ms(5);
    // vbus_off();
    hdr = PD_HEADER(PD_CTRL_PS_RDY, 1, 1, 0, 0, PD_REV20, 0);
    fusb302_tcpm_transmit(0, TCPC_TX_SOP, hdr, NULL);
    mprintf(usb_pd, ">PS_RDY\r\n");
}

static void parse_request(uint32_t value)
{
    // Check reserved bits
    if (value & (1 << 31) || (value & 0x00F00000))
    {
        uprintf("Error: Reserved bits are not set to zero.\r\n");
        return;
    }

    // Extract object position
    uint32_t objectPosition = (value >> 28) & 0x07;
    if (objectPosition == 0)
    {
        uprintf("Error: Object position is set to a reserved value.\r\n");
        return;
    }

    // Extract flags
    uint32_t giveBackFlag = (value >> 27) & 0x01;
    uint32_t capabilityMismatch = (value >> 26) & 0x01;
    uint32_t usbCommCapable = (value >> 25) & 0x01;
    uint32_t noUSBSuspend = (value >> 24) & 0x01;

    // Extract currents
    uint32_t operatingCurrent = (value >> 10) & 0x03FF; // 10 bits for operating current
    uint32_t maxOperatingCurrent = value & 0x03FF;      // 10 bits for max operating current

    // Print details
    uprintf("\tObject Position: %u\r\n", objectPosition);
    uprintf("\tGiveBack Flag: %u\r\n", giveBackFlag);
    uprintf("\tCapability Mismatch: %u\r\n", capabilityMismatch);
    uprintf("\tUSB Communications Capable: %u\r\n", usbCommCapable);
    uprintf("\tNo USB Suspend: %u\r\n", noUSBSuspend);
    uprintf("\tOperating Current: %u mA\r\n", operatingCurrent * 10);
    uprintf("\tMaximum Operating Current: %u mA\r\n", maxOperatingCurrent * 10);
}

static void handle_msg(tamarin_usb_pd *usb_pd, enum fusb302_rxfifo_tokens sop, int hdr, uint32_t *msg)
{
    int len = PD_HEADER_CNT(hdr);
    int type = PD_HEADER_TYPE(hdr);

    if (len != 0)
    {
        switch (type)
        {
        case PD_DATA_SOURCE_CAP:
            mprintf(usb_pd, "<SOURCE_CAP: %02X\r\n", msg[0]);
            break;
        case PD_DATA_REQUEST:
            mprintf(usb_pd, "<REQUEST: 0x%08X\r\n", msg[0]);
            if (usb_pd->log_messages)
            {
                parse_request(msg[0]);
            }
            handle_power_request(usb_pd, msg[0]);
            break;
        case PD_DATA_VENDOR_DEF:
            mprintf(usb_pd, "<VENDOR_DEF\r\n");
            handle_vdm(usb_pd, sop, hdr, msg);
            break;
        default:
            mprintf(usb_pd, "<UNK DATA ");
            dump_msg(usb_pd, sop, hdr, msg);
            break;
        }
    }
    else
    {
        switch (type)
        {
        case PD_CTRL_ACCEPT:
            mprintf(usb_pd, "<ACCEPT\r\n");
            break;
        case PD_CTRL_REJECT:
            mprintf(usb_pd, "<REJECT\r\n");
            break;
        case PD_CTRL_PS_RDY:
            mprintf(usb_pd, "<PS_RDY\r\n");
            break;
        case PD_CTRL_PR_SWAP:
            mprintf(usb_pd, "<PR_SWAP\r\n");
            break;
        case PD_CTRL_DR_SWAP:
            mprintf(usb_pd, "<DR_SWAP\r\n");
            break;
        case PD_CTRL_GET_SINK_CAP:
            mprintf(usb_pd, "<GET_SINK_CAP\r\n");
            break;
        default:
            mprintf(usb_pd, "<UNK CTL ");
            dump_msg(usb_pd, sop, hdr, msg);
            break;
        }
    }
}
static void usb_pd_reinitialize(tamarin_usb_pd *usb_pd)
{
    uprintf("reinitialize\r\n");
    vbus_off(usb_pd);
    fusb302_pd_reset(0);
    fusb302_tcpm_set_msg_header(0, 1, 1); // Source
    fusb302_tcpm_set_cc(0, TYPEC_CC_RP);
    fusb302_auto_goodcrc_enable(0, 1);
    sleep_ms(500);
    fusb302_tcpm_set_vconn(0, 1);
    fusb302_tcpm_set_rx_enable(0, 0);
    // Set CC resistor pull value to USB (can also be 1.5A, 3A, etc)
    fusb302_tcpm_select_rp_value(0, POWER);
    fusb302_tcpm_set_cc(0, TYPEC_CC_RP); // DFP mode
    fusb302_setup_mdac();
    fusb302_setup_toggle();
    fusb302_enable_toggle();

    vbus_on(usb_pd);
    usb_pd->state = USB_STATE_WAITING_FOR_COMP_CHNG;
}

static void usb_pd_interrupt_callback(uint gpio, uint32_t events)
{
    tamarin_usb_pd *usb_pd = interrupt_usb_instance;
    usb_pd->interrupt_pending = true;
    usb_pd_disable_interrupt(usb_pd);
}

void usb_pd_handle_interrupt(tamarin_usb_pd *usb_pd)
{
    if (!usb_pd->interrupt_pending)
    {
        return;
    }
    usb_pd->interrupt_pending = false;
    int16_t irq = 0, irqa = 0, irqb = 0;
    usb_pd_get_interrupt(usb_pd, &irq, &irqa, &irqb);

    // We separately check for hard-resets
    if (irqa & TCPC_REG_INTERRUPTA_HARDRESET)
    {
        usb_pd_reinitialize(usb_pd);
        usb_pd_enable_interrupt(usb_pd);
        return;
    }
    switch (usb_pd->state)
    {
    case USB_STATE_WAITING_FOR_COMP_CHNG:
        if ((irq & TCPC_REG_INTERRUPT_COMP_CHNG) || (irqa & TCPC_REG_INTERRUPTA_TOGDONE) || (irq & TCPC_REG_INTERRUPT_BC_LVL))
        {
            uprintf("🔌Plug-event detected!\r\n");
            // TODO: This is currently disabled and hard-coded to
            //       a single polarity.
            fusb302_disable_toggle();
            int16_t cc1 = fusb302_measure_cc_pin_source(0, 0);
            int16_t cc2 = fusb302_measure_cc_pin_source(0, 1);
            uprintf("CCs: %d %d\r\n", cc1, cc2);
            // if(cc1 && !cc2) {
            //     uprintf("CC2 active\r\n");
            //     // polarity = CC_POLARITY_CC1;
            //     // fusb302_tcpm_set_polarity(0, 0);
            //     // int16_t fusb302_tcpm_set_polarity(int16_t port, int16_t polarity)

            // } else if(!cc1 && cc2) {
            //     uprintf("CC1 active\r\n");
            //     // fusb302_tcpm_set_polarity(0, 1);
            //     // polarity = CC_POLARITY_CC2;
            // } else {
            //     uprintf("Invalid CC: %d %d\r\n", cc1, cc2);
            //     // uprintf("Halting :(\r\n");
            //     // while(1) {}
            // }
            fusb302_tcpm_set_cc(0, TYPEC_CC_RP);
            fusb302_tcpm_set_polarity(0, 1);
            
            sleep_ms(200);
            fusb302_tcpm_set_rx_enable(0, 1);
            fusb302_pd_reset(0);
            fusb302_tcpm_set_msg_header(0, 1, 1); // Source
            send_source_cap(usb_pd);
            usb_pd->state = USB_STATE_WAIT_REQUEST;
        }
        break;
    case USB_STATE_WAIT_REQUEST:
        if (irqb & TCPC_REG_INTERRUPTB_GCRCSENT)
        {
            int16_t hdr, ret;
            enum fusb302_rxfifo_tokens sop;
            uint32_t msg[16];

            ret = fusb302_tcpm_get_message(0, msg, &hdr, &sop);
            if (ret)
            {
                uprintf("Message not received!NoCRC!!!\r\n");
                // No packet or GoodCRC
                break;
            }
            handle_msg(usb_pd, sop, hdr, msg);
            usb_pd->initialized_callback(usb_pd);
            usb_pd->state = USB_STATE_IDLE;
        }
        // } else if (irqa & TCPC_REG_INTERRUPTA_RETRYFAIL)
        // {
        //     // Failed to send source-cap - lets try again.
        //     usb_pd->state = USB_STATE_WAITING_FOR_COMP_CHNG;
        //     uprintf("Retryfail during sending of source_cap\r\n");
        //     usb_pd_reinitialize(usb_pd);
        // }
        break;
    case USB_STATE_IDLE:
        if(usb_pd->handle_messages_in_idle) {
            if (irqb & TCPC_REG_INTERRUPTB_GCRCSENT)
            {
                int16_t hdr, ret;
                enum fusb302_rxfifo_tokens sop;
                uint32_t msg[16];

                ret = fusb302_tcpm_get_message(0, msg, &hdr, &sop);
                if (ret)
                {
                    uprintf("Message not received!NoCRC!!!\r\n");
                    // No packet or GoodCRC
                    break;
                }
                handle_msg(usb_pd, sop, hdr, msg);
            }
        }
        break;
    }
    usb_pd_enable_interrupt(usb_pd);
}

void usb_pd_enable_interrupt(tamarin_usb_pd *usb_pd)
{
    interrupt_usb_instance = usb_pd;
    // First, check if the line is low so we don't
    // miss an interrupt.
    if (!gpio_get(usb_pd->pin_int))
    {
        usb_pd->interrupt_pending = true;
    }
    gpio_set_irq_enabled_with_callback(usb_pd->pin_int, GPIO_IRQ_EDGE_FALL, true, &usb_pd_interrupt_callback);
}

void usb_pd_disable_interrupt(tamarin_usb_pd *usb_pd)
{
    gpio_set_irq_enabled_with_callback(usb_pd->pin_int, GPIO_IRQ_EDGE_FALL, false, &usb_pd_interrupt_callback);
}

bool usb_pd_init(tamarin_usb_pd *usb_pd, uint32_t pin_scl, uint32_t pin_sda, uint32_t pin_int)
{
    usb_pd->interrupt_pending = false;
    usb_pd->i2c = i2c0;
    usb_pd->pin_scl = pin_scl;
    usb_pd->pin_sda = pin_sda;
    usb_pd->pin_int = pin_int;
    usb_pd->state = USB_STATE_NONE;
    usb_pd->initialized_callback = NULL;

    // Initialize FUSB interrupt pin
    gpio_init(usb_pd->pin_int);
    gpio_pull_up(usb_pd->pin_int);
    gpio_set_dir(usb_pd->pin_int, false);

    i2c_init(usb_pd->i2c, 50 * 1000);
    gpio_set_function(usb_pd->pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(usb_pd->pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(usb_pd->pin_sda);
    gpio_pull_up(usb_pd->pin_scl);

    // Try to read the device ID to check if we can talk to the FUSB302
    int16_t reg;
    tcpc_read(0, TCPC_REG_DEVICE_ID, &reg);
    uprintf("Device ID: %d\r\n", reg);
    if (!(reg & 0x80))
    {
        uprintf("Invalid device ID %04x. Is the FUSB302 alive?\r\n", reg);
        usb_pd->state = USB_STATE_FUSB_NOT_FOUND;
        return 1;
    }

    uprintf("Device ID: %c_rev%c (0x%x)\r\n",
           'A' + ((reg >> 4) & 0x7), 'A' + (reg & 3), reg);

    fusb302_tcpm_init(0);
    fusb302_pd_reset(0);
    // Power & data source
    fusb302_tcpm_set_msg_header(0, 1, 1);
    // Configure CC pulls
    fusb302_tcpm_set_cc(0, TYPEC_CC_RP);
    fusb302_auto_goodcrc_enable(0, 1);
    vbus_off(usb_pd);
    sleep_ms(500);
    // fusb302_tcpm_set_vconn(0, 1);
    fusb302_tcpm_set_rx_enable(0, 0);
    // Set CC resistor pull value to USB (can also be 1.5A, 3A, etc)
    fusb302_tcpm_select_rp_value(0, POWER);
    // DFP = Downstream facing port
    fusb302_tcpm_set_cc(0, TYPEC_CC_RP); // DFP mode

    // Configure the DAC so we can detect plug events
    fusb302_setup_mdac();
    // Setup the toggle auto-detect functionality
    fusb302_setup_toggle();
    // Enable plug auto-detect
    fusb302_enable_toggle();
    // Enable the power-supply
    vbus_on(usb_pd);

    usb_pd->state = USB_STATE_WAITING_FOR_COMP_CHNG;
    // Enable interrupt
    usb_pd_enable_interrupt(usb_pd);
    return 0;
}
