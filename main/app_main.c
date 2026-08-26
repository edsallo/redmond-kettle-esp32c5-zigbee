#include "esp_check.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "kettle_ble.h"
#include "zigbee_bridge.h"
#include "status_panel.h"

static const char *TAG = "RK_G211S_BRIDGE";

static void diagnostic_console_task(void *arg)
{
    char line[32];
    ESP_LOGI(TAG, "Console ready: heat 40..100 | boil | off");
    for (;;) {
        if (!fgets(line, sizeof(line), stdin)) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (strncmp(line, "heat ", 5) == 0) {
            int target = atoi(line + 5);
            esp_err_t err = kettle_ble_heat_to((uint8_t)target);
            ESP_LOGI(TAG, "CONSOLE heat %d: %s", target, esp_err_to_name(err));
        } else if (strncmp(line, "boil", 4) == 0) {
            esp_err_t err = kettle_ble_set_power(true);
            ESP_LOGI(TAG, "CONSOLE boil: %s", esp_err_to_name(err));
        } else if (strncmp(line, "off", 3) == 0) {
            esp_err_t err = kettle_ble_set_power(false);
            ESP_LOGI(TAG, "CONSOLE off: %s", esp_err_to_name(err));
        } else {
            ESP_LOGW(TAG, "Unknown command. Use: heat 40..100 | boil | off");
        }
    }
}

static void kettle_status_changed(const r4s_status_t *status, bool available)
{
    zigbee_bridge_update(status, available);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "Starting REDMOND BLE to Zigbee bridge");
    ESP_ERROR_CHECK(zigbee_bridge_start());
    ESP_ERROR_CHECK(kettle_ble_start(kettle_status_changed));
    ESP_ERROR_CHECK(status_panel_start());
    configASSERT(xTaskCreate(diagnostic_console_task, "console", 3072, NULL, 4, NULL) == pdPASS);
}
