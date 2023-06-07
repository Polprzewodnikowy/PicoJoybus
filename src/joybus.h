#pragma once


#include <stdbool.h>
#include <stdint.h>

#include <hardware/pio.h>


#define JOYBUS_BUFFER_SIZE      (64)


enum joybus_cmd_e {
    JOYBUS_CMD_INFO = 0x00,
    JOYBUS_CMD_STATE = 0x01,
    JOYBUS_CMD_READ = 0x02,
    JOYBUS_CMD_WRITE = 0x03,
    JOYBUS_CMD_RESET = 0xFF,
};


typedef uint8_t (*joybus_callback_t) (uint8_t ch, uint8_t cmd, uint8_t rx_length, uint8_t *rx_buffer, uint8_t *tx_buffer);


void joybus_init (PIO pio, uint8_t channels, uint *pins, joybus_callback_t callback);
bool joybus_check_address_crc (uint16_t address);
uint8_t joybus_calculate_data_crc (uint8_t *buffer);
