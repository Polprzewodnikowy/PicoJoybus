#include <btstack_tlv.h>
#include <inttypes.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>
#include <stdio.h>

#include "btstack_config.h"
#include "btstack.h"
#include "joybus.h"


static volatile uint32_t controller_state = 0;
static uint8_t controller_pak[32 * 1024];
static volatile bool rumble_on = false;
static enum { ACC_NONE, ACC_CPAK, ACC_RPAK } accessory_inserted = ACC_RPAK;

static uint8_t __time_critical_func(joybus_callback) (uint8_t ch, uint8_t cmd, uint8_t rx_length, uint8_t *rx_buffer, uint8_t *tx_buffer) {
    uint8_t tx_length = 0;
    uint16_t address_with_crc = (rx_buffer[0] << 8) | rx_buffer[1];
    uint16_t address = address_with_crc & 0xFFE0;

    if (ch > 0) {
        return 0;
    }

    switch (cmd) {
        case JOYBUS_CMD_INFO:
        case JOYBUS_CMD_RESET:
            if (rx_length == 0) {
                uint32_t info = 0x050000 | ((accessory_inserted == ACC_NONE) ? 0x02 : 0x01);

                tx_length = 3;
                tx_buffer[0] = ((info >> 16) & 0xFF);
                tx_buffer[1] = ((info >> 8) & 0xFF);
                tx_buffer[2] = (info & 0xFF);
            }
            break;

        case JOYBUS_CMD_STATE:
            if (rx_length == 0) {
                tx_length = 4;
                tx_buffer[0] = ((controller_state >> 24) & 0xFF);
                tx_buffer[1] = ((controller_state >> 16) & 0xFF);
                tx_buffer[2] = ((controller_state >> 8) & 0xFF);
                tx_buffer[3] = (controller_state & 0xFF);
            }
            break;

        case JOYBUS_CMD_READ:
            if ((rx_length == 2) && joybus_check_address_crc(address_with_crc)) {
                tx_length = 33;
                switch (accessory_inserted) {
                    case ACC_NONE:
                        memset(tx_buffer, 0x00, 32);
                        break;

                    case ACC_CPAK:
                        if (address < 32 * 1024) {
                            memcpy(tx_buffer, controller_pak + address, 32);
                        } else {
                            memset(tx_buffer, 0x00, 32);
                        }
                        break;

                    case ACC_RPAK:
                        memset(tx_buffer, (address & 0xC000) == 0x8000 ? 0x80 : 0x00, 32);
                        break;
                }
                tx_buffer[32] = joybus_calculate_data_crc(tx_buffer);
            }
            break;

        case JOYBUS_CMD_WRITE:
            if ((rx_length == 34) && joybus_check_address_crc(address_with_crc)) {
                switch (accessory_inserted) {
                    case ACC_NONE:
                        break;

                    case ACC_CPAK:
                        if (address < 32 * 1024) {
                            memcpy(controller_pak + address, rx_buffer + 2, 32);
                        }
                        break;

                    case ACC_RPAK:
                        if ((address & 0xC000) == 0xC000) {
                            rumble_on = rx_buffer[33] & 0x01;
                        }
                        break;
                }

                tx_length = 1;
                tx_buffer[0] = joybus_calculate_data_crc(rx_buffer + 2);
            }
            break;

        default:
            printf("Unknown command received: 0x%02X, length: %d\n", cmd, rx_length);
            printf_hexdump(rx_buffer, rx_length);
            break;
    }

    return tx_length;
}


static uint16_t hids_cid;

static void hid_handle_input_report (uint8_t service_index, const uint8_t *report, uint16_t report_len) {
    if (report_len == 0) {
        return;
    }

    const uint8_t *hid_descriptor = hids_client_descriptor_storage_get_descriptor_data(hids_cid, service_index);
    uint16_t hid_descriptor_len = hids_client_descriptor_storage_get_descriptor_len(hids_cid, service_index);

    btstack_hid_parser_t parser;
    btstack_hid_parser_init(&parser, hid_descriptor, hid_descriptor_len, HID_REPORT_TYPE_INPUT, report, report_len);

    uint16_t usage_page;
    uint16_t usage;
    int32_t value;

    int32_t tmp_value;
    uint32_t tmp_state = 0;
    bool configuration_mode = false;

    while (btstack_hid_parser_has_more(&parser)) {
        btstack_hid_parser_get_field(&parser, &usage_page, &usage, &value);

        switch (usage_page) {
            case 0x01: // Generic Desktop Page
                switch (usage) {
                    case 0x30: // X
                        tmp_value = ((value - 0x8000) / 0x199);
                        tmp_state |= (((uint8_t) (tmp_value)) << 8); // X
                        break;
                    case 0x31: // Y
                        tmp_value = -(((value - 0x8000) / 0x199));
                        tmp_state |= (((uint8_t) (tmp_value)) << 0); // Y
                        break;
                    case 0x32: // Z
                        tmp_value = value - 0x8000;
                        if (tmp_value < -0x3FFF) {
                            tmp_state |= (1 << 17); // CL
                        }
                        if (tmp_value > 0x3FFF) {
                            tmp_state |= (1 << 16); // CR
                        }
                        break;
                    case 0x35: // Rz
                        tmp_value = value - 0x8000;
                        if (tmp_value < -0x3FFF) {
                            tmp_state |= (1 << 19); // CU
                        }
                        if (tmp_value > 0x3FFF) {
                            tmp_state |= (1 << 18); // CD
                        }
                        break;
                    case 0x39: // Hat Switch
                        if (value == 8 || value == 1 || value == 2) {
                            tmp_state |= (1 << 27); // DU
                        }
                        if (value == 2 || value == 3 || value == 4) {
                            tmp_state |= (1 << 24); // DR
                        }
                        if (value == 4 || value == 5 || value == 6) {
                            tmp_state |= (1 << 26); // DD
                        }
                        if (value == 6 || value == 7 || value == 8) {
                            tmp_state |= (1 << 25); // DL
                        }
                        break;
                }
                break;

            case 0x02: // Simulation Controls Page
                switch (usage) {
                    case 0xC4: // Accelerator
                        if (value > 0x3F) {
                            tmp_state |= (1 << 20); // R
                        }
                        break;
                    case 0xC5: // Brake
                        if (value > 0x3F) {
                            tmp_state |= (1 << 29); // Z
                        }
                        break;
                }
                break;

            case 0x09: // Button Page
                switch (usage) {
                    case 0x01: // A
                        tmp_state |= (value << 31); // A
                        break;
                    case 0x02: // B
                        tmp_state |= (value << 18); // CD
                        break;
                    case 0x04: // X
                        tmp_state |= (value << 30); // B
                        break;
                    case 0x05: // Y
                        tmp_state |= (value << 19); // CU
                        break;
                    case 0x07: // L
                        tmp_state |= (value << 17); // CL
                        break;
                    case 0x08: // R
                        tmp_state |= (value << 16); // CR
                        break;
                    case 0x0B: // Option
                        tmp_state |= (value << 28); // Start
                        break;
                    case 0x0C: // Menu
                        tmp_state |= (value << 28); // Start
                        break;
                    case 0x0D: // XBOX
                        // N64RGB reset combination (A + B + Z + Start + R)
                        tmp_state |= value ? ((1 << 31) | (1 << 30) | (1 << 29) | (1 << 28) | (1 << 20)) : 0;
                        break;
                    case 0x0E: // LS
                        tmp_state |= (value << 21); // L
                        break;
                    case 0x0F: // RS
                        break;
                }
                break;

            case 0x0C: // Consumer Page
                switch (usage) {
                    case 0xB2: // Record
                        configuration_mode = value;
                        break;
                }
                break;
        }
    }

    if (configuration_mode) {
        if (tmp_state & (1 << 26)) { // DD
            accessory_inserted = ACC_NONE;
        } else if (tmp_state & (1 << 25)) { // DL
            accessory_inserted = ACC_RPAK;
        } else if (tmp_state & (1 << 24)) { // DR
            accessory_inserted = ACC_CPAK;
        }

        controller_state = 0;
    } else {
        // Handle L + R + Start button combo
        if ((tmp_state & ((1 << 28) | (1 << 21) | (1 << 20))) == ((1 << 28) | (1 << 21) | (1 << 20))) {
            tmp_state &= ~(1 << 28); // Clear "Start" button
            tmp_state |= (1 << 23); // Set hidden "Reset" button
        }

        controller_state = tmp_state;
    }
}


static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META) {
        return;
    }

    switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
        case GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED: {
            if (gattservice_subevent_hid_service_connected_get_status(packet) == ERROR_CODE_SUCCESS) {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            }
            break;
        }

        case GATTSERVICE_SUBEVENT_HID_REPORT: {
            uint8_t service_index = gattservice_subevent_hid_report_get_service_index(packet);
            const uint8_t *report = gattservice_subevent_hid_report_get_report(packet);
            uint16_t report_len = gattservice_subevent_hid_report_get_report_len(packet);
            hid_handle_input_report(service_index, report, report_len);
            break;
        }
    }
}


static const uint8_t xbox_address[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Fill that yourself

static void hci_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE: {
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                // btstack / pico impl bug?
                le_connection_parameter_range_t range;
                range.le_conn_interval_min = 0;
                range.le_conn_interval_max = 0;
                range.le_conn_latency_min = 0;
                range.le_conn_latency_max = 0;
                range.le_supervision_timeout_min = 0;
                range.le_supervision_timeout_max = 0;
                gap_set_connection_parameter_range(&range);

                gap_set_connection_parameters(96, 96, 6, 12, 0, 300, 0, 0);

                gap_whitelist_clear();
                gap_whitelist_add(BD_ADDR_TYPE_LE_PUBLIC, xbox_address);
                gap_connect_with_whitelist();
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            hci_con_handle_t handle = hci_event_disconnection_complete_get_connection_handle(packet);
            hids_client_disconnect(hids_cid);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            controller_state = 0;
            gap_connect_with_whitelist();
            break;
        }

        case HCI_EVENT_LE_META: {
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE: {
                    if (hci_subevent_le_connection_complete_get_status(packet) != ERROR_CODE_SUCCESS) {
                        break;
                    }
                    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
                    sm_request_pairing(hci_subevent_le_connection_complete_get_connection_handle(packet));
                    break;
                }
            }
            break;
        }

        case HCI_EVENT_ENCRYPTION_CHANGE: {
            if (hci_event_encryption_change_get_encryption_enabled(packet)) {
                hci_con_handle_t handle = hci_event_encryption_change_get_connection_handle(packet);
                if (hids_client_connect(handle, handle_gatt_client_event, HID_PROTOCOL_MODE_REPORT, &hids_cid) != ERROR_CODE_SUCCESS) {
                    gap_disconnect(handle);
                }
            }
            break;
        }
    }
}


static void sm_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
        case SM_EVENT_JUST_WORKS_REQUEST: {
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        }

        case SM_EVENT_PAIRING_COMPLETE: {
            switch (sm_event_pairing_complete_get_status(packet)) {
                case ERROR_CODE_SUCCESS: {
                    break;
                }

                default: {
                    gap_disconnect(sm_event_pairing_complete_get_handle(packet));
                    break;
                }
            }
            break;
        }

        case SM_EVENT_REENCRYPTION_COMPLETE: {
            switch (sm_event_reencryption_complete_get_status(packet)) {
                case ERROR_CODE_SUCCESS: {
                    break;
                }

                case ERROR_CODE_PIN_OR_KEY_MISSING: {
                    uint8_t address[6];
                    sm_event_reencryption_complete_get_address(packet, address);
                    gap_delete_bonding(sm_event_reencryption_started_get_addr_type(packet), address);
                    sm_request_pairing(sm_event_reencryption_complete_get_handle(packet));
                    break;
                }

                default: {
                    gap_disconnect(sm_event_reencryption_complete_get_handle(packet));
                    break;
                }
            }
            break;
        }
    }
}


static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

static uint8_t hid_descriptor_storage[512];

static uint8_t fb_on[8] = {
    0x03, // enable
    0x00, // mag_lt
    0x00, // mag_rt
    0x1E, // mag_l
    0x1E, // mag_r
    0xFF, // duration
    0x00, // delay
    0x00, // cnt
};

static uint8_t fb_off[8] = {
    0x03, // enable
    0x00, // mag_lt
    0x00, // mag_rt
    0x00, // mag_l
    0x00, // mag_r
    0xFF, // duration
    0x00, // delay
    0xFF, // cnt
};

int main (void) {
    stdio_init_all();

    uint joybus_pins[1] = { 28 };
    joybus_init(pio1, 1, joybus_pins, joybus_callback);

    if (cyw43_arch_init()) {
        printf("cyw43_arch_init failed\n");
        return -1;
    }

    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();

    sm_init();
    sm_set_accepted_stk_generation_methods(SM_STK_GENERATION_METHOD_JUST_WORKS);
    sm_set_encryption_key_size_range(7, 16);
    sm_set_authentication_requirements(SM_AUTHREQ_BONDING);
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    gatt_client_init();

    hids_client_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));

    hci_power_control(HCI_POWER_ON);

    bool rumble_state = false;

    while (true) {
        if (rumble_state ^ rumble_on) {
            rumble_state = !rumble_state;
            while (true) {
                // FIXME: There's bug in the btstack that prevents this function from working more than once
                // btstack/src/ble/gatt-service/hids_client.c:813 - callback parameter in `gatt_client_write_value_of_characteristic` should be `handle_gatt_client_event` instead of `handle_report_event`
                if (hids_client_send_write_report(hids_cid, 3, HID_REPORT_TYPE_OUTPUT, rumble_state ? fb_on : fb_off, 8) == ERROR_CODE_SUCCESS) {
                    break;
                }
            }
        }
    }
}
