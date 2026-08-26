#pragma once

#include "esp_err.h"
#include "redmond_protocol.h"

typedef void (*kettle_status_callback_t)(const r4s_status_t *status, bool available);

esp_err_t kettle_ble_start(kettle_status_callback_t callback);
esp_err_t kettle_ble_set_power(bool on);
esp_err_t kettle_ble_heat_to(uint8_t temperature);
bool kettle_ble_is_connected(void);
void kettle_ble_reconnect(void);
void kettle_ble_forget_and_reconnect(void);
