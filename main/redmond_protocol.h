#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define R4S_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define R4S_RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define R4S_TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

typedef enum {
    R4S_CMD_ON       = 0x03,
    R4S_CMD_OFF      = 0x04,
    R4S_CMD_SET_MODE = 0x05,
    R4S_CMD_STATUS   = 0x06,
    R4S_CMD_COMMIT   = 0x36,
    R4S_CMD_SET_SWITCH = 0x37,
    R4S_CMD_SOUND    = 0x3c,
    R4S_CMD_TIME_SYNC = 0x6e,
    R4S_CMD_AUTH     = 0xff,
} r4s_command_t;

typedef struct {
    uint8_t mode;
    uint8_t target_temperature;
    uint8_t temperature;
    bool is_on;
    uint8_t error;
} r4s_status_t;

size_t r4s_make_packet(uint8_t counter, uint8_t command,
                       const uint8_t *payload, size_t payload_len,
                       uint8_t *out, size_t out_size);
void r4s_make_mode_payload(bool keep_warm, uint8_t target, uint8_t out[16]);
bool r4s_decode_status(const uint8_t *payload, size_t length, r4s_status_t *status);
