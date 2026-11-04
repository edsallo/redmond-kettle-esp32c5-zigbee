#pragma once

#include "esp_err.h"
#include <stdint.h>

typedef enum {
    WIFI_OTA_IDLE = 0,
    WIFI_OTA_CONNECTING = 1,
    WIFI_OTA_DOWNLOADING = 2,
    WIFI_OTA_INSTALLING = 3,
    WIFI_OTA_SUCCESS = 4,
    WIFI_OTA_FAILED = 5,
} wifi_ota_status_t;

typedef void (*wifi_ota_status_cb_t)(wifi_ota_status_t status, uint8_t progress);

esp_err_t wifi_ota_init(wifi_ota_status_cb_t callback);
esp_err_t wifi_ota_save_config(const char *ssid, const char *password, const char *url);
esp_err_t wifi_ota_start(void);
const char *wifi_ota_version(void);
