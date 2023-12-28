/*  
    Tamarin-C - Tool for exploring USB-C on iPhones & macs
    Copyright 2023 Thomas 'stacksmashing' Roth - code@stacksmashing.net

    Heavily based on previous work by the AsahiLinux project, Marc Zyngier, etc.
    Check out: vdmtool and macvdmtool

    Released under GPLv3. See LICENSE for details.
*/

// Pico includes
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"

// Tiny USB
#include "bsp/board.h"
#include "tusb.h"

// Tamarin specific
#include "tamarin_hw.h"
#include "usb_pd_tcpm.h"
#include "tcpm_driver.h"
#include "usb_pd.h"
#include "tamarin_probe.h"
#include "uart_rx.pio.h"

#include <stdlib.h>

enum CC_POLARITY polarity = CC_POLARITY_CC1;

enum PIN_MAPPING
{
    PIN_MAPPING_NONE = 0,
    PIN_MAPPING_DPDN1 = 1,
    PIN_MAPPING_DPDN2 = 2,
    PIN_MAPPING_SBU = 4
};

const bool ACTION_ENTER = 0;

tamarin_usb_pd usb_pd = {
    .log_interrupts = false,
    .log_messages = false,
    // Needs to be true for USB-C -> Lightning
    .handle_messages_in_idle = false
};

// Mixes config and state a bit
typedef struct tamarin_configuration {
    // Config
    enum PIN_MAPPING uart_pins;
    enum PIN_MAPPING bus1_pins;
    enum PIN_MAPPING bus2_pins;
    enum PIN_MAPPING swd_pins;
    bool uart_on_initialize;
    bool probe_on_initialize;

    // State
    tamarin_usb_pd *usb_pd;
    bool probe_initialized;
    bool uart_initialized;

    PIO pio;
    uint sm;
} tamarin_configuration;

tamarin_configuration config = {
    .uart_pins = PIN_MAPPING_DPDN2,
    .bus1_pins = PIN_MAPPING_SBU,
    .bus2_pins = PIN_MAPPING_DPDN2,
    .swd_pins = PIN_MAPPING_SBU,
    .uart_on_initialize = false,
    .probe_on_initialize = false,
    .usb_pd = &usb_pd,
    .probe_initialized = false,
    .uart_initialized = false,
    .pio = pio0,
    .sm = 0
};

char *MAPPING_TO_STRING[] = {
    "None",
    "D+/D- 1",
    "D+/D- 2",
    "INVALID",
    "SBU"
};

struct known_actions {
    int id;
    char *name;
};

struct known_actions known_actions[] = {
    {.id = 0x206, .name="SWD"},
    {.id = 0x306, .name="UART"},
    {.id = 0x103, .name = "PD Reset"},
    {.id = 0x105, .name = "Reboot"},
    {.id = 0x106, .name = "DFU / Hold"},
    {.id = 0x606, .name = "DFU USB"},
    {.id = 0x4606, .name = "Debug USB"},
    {.id = 0x0803, .name = "SPMI/I2C"},
    {.id = 0x0809, .name = "Unkn/I2C"},
};

void cmd_jtag_mode(tamarin_configuration *config);
void cmd_uart_mode(tamarin_configuration *config);

char *get_action_description(int action_id) {
    for(int i=0; i < sizeof(known_actions)/sizeof(struct known_actions); i++) {
        if(known_actions[i].id == action_id) {
            return known_actions[i].name;
        }
    }

    return "";
}

static void vdm_send_msg(tamarin_usb_pd *usb_pd, uint32_t *vdm, int nr_u32)
{
    if(usb_pd->log_messages) {
        uprintf("Sending (%d): ", nr_u32);
        for (int i = 0; i < nr_u32; i++)
        {
            uprintf("0x%08X ", vdm[i]);
        }
        uprintf("\r\n");
    }

    int16_t hdr = PD_HEADER(PD_DATA_VENDOR_DEF, 1, 1, 0, nr_u32, PD_REV20, 0);
    fusb302_tcpm_transmit(0, TCPC_TX_SOP_DEBUG_PRIME_PRIME, hdr, vdm);
}

const bool ACTION_EXIT = 1;
void vdm_apple_perform_action(tamarin_usb_pd *usb_pd, bool exit, bool persist, bool exit_conflicting, enum PIN_MAPPING mapping, uint16_t action_id, uint32_t arguments[], size_t arguments_len)
{
    uint8_t line_config = mapping;
    uint32_t action =
        (exit << 25) |
        (persist << 24) |
        (exit_conflicting << 23) |
        (line_config << 16) |
        action_id;

    // 2 = command + action
    uint32_t message_length = 2 + arguments_len;

    if(arguments_len > 14) {
        printf("Error: Maximum argument length is 14\r\n");
        return;
    }
    uint32_t vdm[16];
    vdm[0] = 0x5ac8012;
    vdm[1] = action;
    for (int i = 0; i < arguments_len; i++)
    {
        vdm[2 + i] = arguments[i];
    }
    vdm_send_msg(usb_pd, vdm, message_length);
}

/* This function does not handle hard-resets etc., it's very naive. */
void vdm_send_msg_blocking(
    tamarin_usb_pd *usb_pd,
    uint32_t vdm[],
    size_t message_length,
    uint32_t out[],
    size_t *out_len,
    uint32_t timeout)
{
    usb_pd_disable_interrupt(usb_pd);
    vdm_send_msg(usb_pd, vdm, message_length);
    // vdm_apple_perform_action(exit, persist, exit_conflicting, mapping, action_id, arguments, arguments_len);
    if(usb_pd->log_messages) {
        uprintf("Waiting for response...\r\n");
    }
    
    uint32_t end_time = to_ms_since_boot(get_absolute_time()) + timeout;
    while (1)
    {
        // Wait for interrupt
        while (gpio_get(PIN_FUSB_INT))
        {
            if((timeout != 0) && (to_ms_since_boot(get_absolute_time()) > end_time)) {
                 uprintf("Error: vdm_send_msg_blocking timed out!\r\n");
                 *out_len = 0 ;
                 usb_pd_enable_interrupt(usb_pd);
                 return;
            }
        }

        // Fetch interrupt
        int16_t irq = 0, irqa = 0, irqb = 0;
        usb_pd_get_interrupt(usb_pd, &irq, &irqa, &irqb);
        if (irqb & TCPC_REG_INTERRUPTB_GCRCSENT)
        {

            int16_t hdr, ret;
            enum fusb302_rxfifo_tokens sop;
            uint32_t msg[16];

            ret = fusb302_tcpm_get_message(0, out, &hdr, &sop);

            int type = PD_HEADER_TYPE(hdr);
            int len = PD_HEADER_CNT(hdr);
            if (ret)
            {
                uprintf("⚠️No packet\r\n");
                // No packet or GoodCRC
                continue;
            }

            if(usb_pd->log_messages) {
                uprintf("<VENDOR");
                for(int i=0; i < len; i++) {
                    uprintf(" %08X", out[i]);
                }
                uprintf("\r\n");
            }
            

            if (type == PD_DATA_VENDOR_DEF)
            {
                *out_len = len;
                usb_pd_enable_interrupt(usb_pd);
                return;
            }
        }
    }
}

void vdm_get_action_info(tamarin_usb_pd *usb_pd, uint16_t action)
{
    uint32_t out[16];
    size_t out_len = 0;
    uint32_t vdm[] = {0x5ac8011, action};

    vdm_send_msg_blocking(usb_pd, vdm, 2, out, &out_len, 1000);
    if (out_len < 2)
    {
        uprintf("Not enough data.\r\n");
        return;
    }
    uprintf("Action: 0x%04X - ", action);
    uprintf("%10s - ", get_action_description(action));
    for (int i = 1; i < out_len; i++)
    {
        uint16_t i1 = out[i] >> 16;
        uint16_t i2 = out[i] & 0xFFFF;
        if (i1 != 0x0)
        {
            uprintf("0x%04X ", i1);
        }
        if (i2 != 0x0)
        {
            uprintf("0x%04X ", i2);
        }
    }
    
    uprintf("\r\n");
}

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

void vdm_pd_reset(tamarin_configuration *config)
{
	uint32_t vdm[] = { 0x5ac8012, 0x0103, 0x8000<<16 };
	vdm_send_msg(config->usb_pd, vdm, ARRAY_SIZE(vdm));
	uprintf(">VDM SET ACTION PD reset\r\n");
}


void vdm_send_reboot(tamarin_configuration *config)
{
    uint32_t arg = 0x8000UL << 16;
    vdm_apple_perform_action(config->usb_pd, 0, 0, 0, PIN_MAPPING_NONE, 0x105, &arg, 1);
    uprintf(">VDM Reboot\r\n");
}

void vdm_dfu_hold(tamarin_configuration *config)
{
    uint32_t arg = 0x8001UL << 16;
    uint32_t vdm[] = { 0x5ac8012, 0x0106, 0x80010000};
    vdm_send_msg(config->usb_pd, vdm, ARRAY_SIZE(vdm));
    uprintf(">VDM DFU (0x0106)\r\n");
}

void vdm_dfu_usb(tamarin_configuration *config)
{
    uint32_t arg = 0x8001UL << 16;
    vdm_apple_perform_action(config->usb_pd, 0,0, 0, PIN_MAPPING_DPDN2, 0x606, &arg, 1);
    uprintf(">VDM DFU USB (0x606)\r\n");
}

void vdm_debug_usb(tamarin_configuration *config)
{
    uint32_t arg = 0x8001UL << 16;
    vdm_apple_perform_action(config->usb_pd, 0, 0, 0, PIN_MAPPING_DPDN2, 0x4606, &arg, 1);
    uprintf(">VDM Debug USB (0x4606)\r\n");
}



void vdm_send_uart(tamarin_configuration *config)
{
    vdm_apple_perform_action(config->usb_pd, 0, 0, 0, config->uart_pins, 0x306, NULL, 0);
    uprintf(">VDM serial on %s\r\n", MAPPING_TO_STRING[config->uart_pins]);
}
void vdm_send_swd(tamarin_configuration *config)
{
    vdm_apple_perform_action(config->usb_pd, 0, 0, 0, config->swd_pins, 0x206, NULL, 0);
    uprintf(">VDM SWD on %s\r\n", MAPPING_TO_STRING[config->swd_pins]);
}

void vdm_enter_bus1(tamarin_configuration *config)
{
    vdm_apple_perform_action(config->usb_pd, 0, 1, 1, config->bus1_pins, 0x0809, NULL, 0);
    uprintf(">VDM BUS1 on %s\r\n", MAPPING_TO_STRING[config->bus1_pins]);
    
}

void vdm_enter_bus2(tamarin_configuration *config)
{
    vdm_apple_perform_action(config->usb_pd, 0, 1, 1, config->bus2_pins, 0x0803, NULL, 0);
    uprintf(">VDM BUS2 on %s\r\n", MAPPING_TO_STRING[config->bus2_pins]);
}

// Unknown on iPhone 15 Pro
void vdm_enter_unknown(tamarin_configuration *config)
{
    vdm_apple_perform_action(config->usb_pd, 0, 0, 0, PIN_MAPPING_SBU, 0x0803, NULL, 0);
    uprintf(">VDM UNKNOWN\r\n");
}

// 6.4MHz UART on iPhone 15 Pro
void vdm_send_weird_uart(tamarin_configuration *config)
{
    vdm_apple_perform_action(config->usb_pd, 0, 1, 1, PIN_MAPPING_DPDN1, 0x0303, NULL, 0);
    uprintf(">VDM WEIRD UART\r\n");
}

// Unknown on iPhone 15 Pro
void vdm_send_weird_uart2(tamarin_configuration *config)
{
    vdm_apple_perform_action(config->usb_pd, 0, 1, 1, PIN_MAPPING_DPDN1, 0x0301, NULL, 0);
    uprintf(">VDM WEIRD UART2\r\n");
    
}

// Unknown on iPhone 15 Pro
void vdm_send_0x0410(tamarin_configuration *config)
{
    vdm_apple_perform_action(config->usb_pd, 0, 0, 1, PIN_MAPPING_SBU, 0x0410, NULL, 0);
    uprintf(">VDM (SBU) 0x0410\r\n");
}


// Unknown on iPhone 15 Pro
void vdm_send_0x0607(tamarin_configuration *config)
{
    vdm_apple_perform_action(config->usb_pd, 0, 0, 1, PIN_MAPPING_SBU, 0x0607, NULL, 0);
    uprintf(">VDM (SBU) 0x0607\r\n");
}

// This is called once a PD negotiation has taken place
void usb_initialized_callback(tamarin_usb_pd *usb_pd)
{
    uprintf("✅Communication initialized\r\n");
    if(config.uart_on_initialize) {
        cmd_uart_mode(&config);
    }
    else if(config.probe_on_initialize)
    {
        cmd_jtag_mode(&config);
    }

    uprintf("\r\n> ");
}

void vdm_get_action_list(tamarin_usb_pd *usb_pd)
{
    uint32_t vdm = 0x5ac8010;
    uint32_t out[32];
    size_t out_len;
    vdm_send_msg_blocking(usb_pd, &vdm, 1, out, &out_len, 1000);
    if (out_len < 2)
    {
        uprintf("Invalid response length!\r\n");
        return;
    }

    uprintf("Supported VDM Actions:\r\n");
    for (int i = 1; i < out_len; i++)
    {
        uint32_t a1 = out[i] >> 16;
        uint32_t a2 = out[i] & 0xFFFF;
        if (a1 != 0x0)
        {
            vdm_get_action_info(usb_pd, a1);
        }
        if (a2 != 0x0)
        {
            vdm_get_action_info(usb_pd, a2);
        }
    }
}


enum MENU_STATES {
    MENU_STATE_MAIN,
    MENU_STATE_MAPPING_SELECT_BUS,
    MENU_STATE_MAPPING_SELECT_MAPPING,
    MENU_STATE_DEFAULT_MODE
};

typedef struct menu {
    enum MENU_STATES state;

    // Used during mapping select to keep the selected bus selection.
    int mapping_selected_bus;
};

struct menu menu = {
    .state = MENU_STATE_MAIN
};

void cmd_jtag_mode(tamarin_configuration *config) {
    if(config->uart_initialized) {
        uprintf("UART already initialized. Currently only one operation type is supported.\r\n");
        return;
    }
    uprintf("Configuring JTAG pins... \r\n");

    vdm_send_swd(config);
    gpio_init(PIN_SHIFTER_SUPPLY);
    gpio_set_dir(PIN_SHIFTER_SUPPLY, 1);
    gpio_put(PIN_SHIFTER_SUPPLY, 1);
    gpio_init(PIN_SBU2_DIR);
    gpio_set_dir(PIN_SBU2_DIR, 1);
    gpio_put(PIN_SBU2_DIR, SHIFTER_DIRECTION_OUT);
    gpio_init(PIN_SBU1_DIR);
    gpio_set_dir(PIN_SBU1_DIR, 1);
    gpio_put(PIN_SBU1_DIR, SHIFTER_DIRECTION_OUT);

    if(config->probe_initialized) {
        tamarin_probe_deinit();
    }
    tamarin_probe_init();
    config->probe_initialized = true;
    
    uprintf("🐞 SWD mode enabled. Use openocd -f interface/tamarin.cfg to connect.\r\n");
}

void cmd_jtag_mode_no_probe(tamarin_configuration *config) {
    uprintf("Configuring JTAG pins... \r\n");
    vdm_send_swd(config);
    uprintf("🐞 SWD mode enabled. Ready for external debugger.\r\n");
}

void cmd_uart_mode(tamarin_configuration *config) {
    if(config->probe_initialized) {
        uprintf("SWD already initialized. Currently only one operation type is supported.\r\n");
        return;
    }
    uprintf("Configuring UART pins... \r\n");

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(config->pio, &uart_rx_program);
    uart_rx_program_init(config->pio, config->sm, offset, PIN_SBU1, 115200);

    gpio_init(PIN_SHIFTER_SUPPLY);
    gpio_set_dir(PIN_SHIFTER_SUPPLY, 1);
    gpio_put(PIN_SHIFTER_SUPPLY, 1);
    gpio_init(PIN_SBU2_DIR);
    gpio_set_dir(PIN_SBU2_DIR, 1);
    gpio_put(PIN_SBU2_DIR, SHIFTER_DIRECTION_IN);
    gpio_init(PIN_SBU1_DIR);
    gpio_set_dir(PIN_SBU1_DIR, 1);
    gpio_put(PIN_SBU1_DIR, SHIFTER_DIRECTION_IN);
    config->uart_initialized = true;
    vdm_send_uart(config);
    uprintf("📜 UART mode enabled. Connect to second serial port for access.\r\n");
}

void cmd_uart_mode_no_probe(tamarin_configuration *config) {
    uprintf("Configuring UART pins... \r\n");
    vdm_send_uart(config);
    uprintf("📜 UART mode enabled. Ready for external probe.\r\n");
}

void cmd_reboot_device(tamarin_configuration *config) {
    vdm_send_reboot(config);
}

void cmd_map_bus1(tamarin_configuration *config) {
    vdm_enter_bus1(config);
}

void cmd_map_bus2(tamarin_configuration *config) {
    vdm_enter_bus2(config);
}

void cmd_map_weird_uart(tamarin_configuration *config) {
    vdm_send_weird_uart(config);
}
void cmd_map_weird_uart2(tamarin_configuration *config) {
    vdm_send_weird_uart2(config);
}

void cmd_map_410(tamarin_configuration *config) {
    vdm_apple_perform_action(config->usb_pd, 0, 1, 1, PIN_MAPPING_DPDN1, 0x0410, NULL, 0);
    uprintf(">VDM 0x410\r\n");
}

void cmd_dfu(tamarin_configuration *config) {
    vdm_dfu_hold(config);

}

void cmd_fetch_actions(tamarin_configuration *config) {
    vdm_get_action_list(config->usb_pd);
    uprintf("\r\n> ");
}

void cmd_set_pin_mapping(tamarin_configuration *config) {
    uprintf("Select mapping to update:\r\n");
    uprintf("- 1 - UART  mapping (Currently: %s)\r\n", MAPPING_TO_STRING[config->uart_pins]);
    uprintf("- 2 - Bus 1 mapping (Currently: %s)\r\n", MAPPING_TO_STRING[config->bus1_pins]);
    uprintf("- 3 - Bus 2 mapping (Currently: %s)\r\n", MAPPING_TO_STRING[config->bus2_pins]);
    uprintf("- 4 - SWD   mapping (Currently: %s)\r\n", MAPPING_TO_STRING[config->swd_pins]);
    uprintf("- C - Cancel\r\n", MAPPING_TO_STRING[config->swd_pins]);
    menu.state = MENU_STATE_MAPPING_SELECT_BUS;
}

void cmd_reset_tamarin(tamarin_configuration *config) {
    watchdog_enable(100, 1);
}

void cmd_firmware_update(tamarin_configuration *config) {
    reset_usb_boot(0, 0);
}

typedef struct tamarin_command {
    char key;
    char *description;
    void (*implementation)(tamarin_configuration *config);
} tamarin_command;

tamarin_command commands[] = {
    {.key = '1', .description = "JTAG Mode (with Tamarin Probe support)", .implementation = cmd_jtag_mode},
    {.key = '!', .description = "JTAG Mode (For external debugger)", .implementation = cmd_jtag_mode_no_probe},
    {.key = '2', .description = "DCSD Mode (with UART in Tamarin)", .implementation = cmd_uart_mode},
    {.key = '@', .description = "DCSD Mode (For external probe)", .implementation = cmd_uart_mode_no_probe},
    {.key = '3', .description = "Reboot device", .implementation = cmd_reboot_device},
    {.key = 'A', .description = "Map internal bus 1 (ACE SPMI on iPhone 15, I2C on macs)", .implementation = cmd_map_bus1},
    {.key = 'B', .description = "Map internal bus 2 (Unknown on iPhone 15, I2C ACE communication on macs)", .implementation = cmd_map_bus2},
    // {.key = 'W', .description = "Map weird UART (iPhone 15) on SBU", .implementation = cmd_map_weird_uart},
    {.key = 'D', .description = "Enter DFU (Debug USB)", .implementation = cmd_dfu},
    {.key = 'F', .description = "Fetch supported VDM actions", .implementation = cmd_fetch_actions},
    // {.key = '4', .description = "Send 0x410 mode (on SBU)", .implementation = cmd_map_410},
    {.key = 'M', .description = "Set pin mapping\r\n", .implementation = cmd_set_pin_mapping},
    {.key = 'R', .description = "Reset Tamarin Cable", .implementation = cmd_reset_tamarin},
    {.key = 'U', .description = "Tamarin firmware update mode", .implementation = cmd_firmware_update},
};


void print_menu() {
    uprintf("\r\nTamarin-C\r\n\r\nHello 37c3!\r\n\r\n");
    for(int i = 0; i < sizeof(commands)/sizeof(tamarin_command); i++) {
        uprintf("%c: %s\r\n", commands[i].key, commands[i].description);
    }
    uprintf("\r\n> ");
    return;
}

void print_available_mappings() {
    uprintf("Select which pins to map to:\r\n");
    uprintf("- 1: D+/D- 1 (Not connected on basically all cables)\r\n");
    uprintf("- 2: D+/D- 2 (Connected through cables)\r\n");
    uprintf("- 3: SBU (Requires Thunderbolt 3+ or USB3 cable)\r\n");
    uprintf("> ");
}

bool probe_initialized = false;
void handle_menu() {
    if(tud_cdc_n_available(ITF_CONSOLE)) {
        int c = tud_cdc_n_read_char(ITF_CONSOLE);
        tud_cdc_n_write_char(ITF_CONSOLE, c);
        switch(menu.state) {
            // Main menu
            case MENU_STATE_MAIN:
                for(int i = 0; i < sizeof(commands)/sizeof(tamarin_command); i++) {
                    tamarin_command *cmd = &commands[i];
                    if(cmd->key == c) {
                        uprintf("\r\n");
                        cmd->implementation(&config);
                        return;
                    }
                }
                print_menu();
                break;
            // Menu to select which bus to update the mapping of
            case MENU_STATE_MAPPING_SELECT_BUS:
                if(('1' <= c) && (c <= '4')) {
                    menu.mapping_selected_bus = c - '1';
                    menu.state = MENU_STATE_MAPPING_SELECT_MAPPING;
                    print_available_mappings();
                } else {
                    cmd_set_pin_mapping(&config);
                }
                break;
            // Menu to select which pins to update the mapping to
            case MENU_STATE_MAPPING_SELECT_MAPPING:
                if(('1' <= c) && (c <= '3')) {
                    int mapping = (c - '1') + PIN_MAPPING_DPDN1;
                    // Special case: Mappings go 1, 2, 4
                    if(mapping == 3) {
                        mapping = 4;
                    }
                    switch(menu.mapping_selected_bus) {
                        case 0:
                            config.uart_pins = mapping;
                            uprintf("\r\nUART mapping updated to: %s\r\n> ", MAPPING_TO_STRING[config.uart_pins]);
                            break;
                        case 1:
                            config.bus1_pins = mapping;
                            uprintf("\r\nBus 1 mapping updated to: %s\r\n> ", MAPPING_TO_STRING[config.uart_pins]);
                            break;
                        case 2:
                            config.bus2_pins = mapping;
                            uprintf("\r\nBus 2 mapping updated to: %s\r\n> ", MAPPING_TO_STRING[config.uart_pins]);
                            break;
                        case 3:
                            config.swd_pins = mapping;
                            uprintf("\r\nSWD mapping updated to: %s\r\n> ", MAPPING_TO_STRING[config.uart_pins]);
                            break;
                        default:
                            uprintf("ERROR: Unsupported selected bus: %d\r\n> ", menu.mapping_selected_bus);
                    }
                    menu.state = MENU_STATE_MAIN;
                } else {
                    print_available_mappings();
                }
                break;
            default:
                printf("ERROR: Unhandled menu state: %d\r\n", menu.state);
        }
    }
}

int main()
{
    board_init();
    tusb_init();


    // Set-up VBUS control
    gpio_init(PIN_VBUS_EN);
    gpio_set_dir(PIN_VBUS_EN, true);

    // Set-up level-shifter power
    // (When the supply is off they should be in low-z)
    gpio_init(PIN_SHIFTER_SUPPLY);
    gpio_set_dir(PIN_SHIFTER_SUPPLY, false);
    gpio_put(PIN_SHIFTER_SUPPLY, 0);

    // LED
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    // Don't do anything until we ahve a console connection
    while(!tud_cdc_n_connected(ITF_CONSOLE)) {
        tud_task();
    }

    gpio_put(25, 1);
    sleep_ms(50);
    gpio_put(25, 0);
    sleep_ms(50);
    print_menu();

    // Initialize USB PD controller
    if(usb_pd_init(&usb_pd, PIN_I2C_SCL, PIN_I2C_SDA, PIN_FUSB_INT)) {
        while(1) {
            gpio_put(25, 1);
            sleep_ms(50);
            gpio_put(25, 0);
            sleep_ms(50);
        }
    }

    // Callback when PD is successfully initialized
    usb_pd.initialized_callback = usb_initialized_callback;

    while (1)
    {
        tud_task();
        
        // This will do nothing if no interrupt is pending.
        // The actual pending flag is set in a GPIO interrupt
        // handler in usb_pd.
        usb_pd_handle_interrupt(&usb_pd);
        handle_menu();
        if(config.probe_initialized) {
            tamarin_probe_task();
        }
        if(config.uart_initialized) {
            if(uart_rx_program_readable(config.pio, config.sm)) {
                char c = uart_rx_program_getc(config.pio, config.sm);
                tud_cdc_n_write_char(ITF_DCSD, c);
                tud_cdc_n_write_flush(ITF_DCSD);
            }
        }
    }
}
