#include "wifi_ota.h"

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs.h"
#include <string.h>

#define CONNECTED_BIT BIT0
#define FAILED_BIT BIT1

static const char *TAG = "WIFI_OTA";
static EventGroupHandle_t s_wifi_events;
static wifi_ota_status_cb_t s_callback;
static bool s_running;
static unsigned s_retries;

static void confirm_image_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(30000));
    ESP_LOGI(TAG, "Confirming OTA image after stability window");
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_mark_app_valid_cancel_rollback());
    vTaskDelete(NULL);
}

static void publish(wifi_ota_status_t status, uint8_t progress)
{
    if (s_callback) s_callback(status, progress);
}

static esp_err_t load_string(nvs_handle_t nvs, const char *key, char *out, size_t size)
{
    size_t required = size;
    return nvs_get_str(nvs, key, out, &required);
}

esp_err_t wifi_ota_save_config(const char *ssid, const char *password, const char *url)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open("wifi_ota", NVS_READWRITE, &nvs), TAG, "open NVS");
    esp_err_t err = nvs_set_str(nvs, "ssid", ssid ? ssid : "");
    if (err == ESP_OK) err = nvs_set_str(nvs, "password", password ? password : "");
    if (err == ESP_OK) err = nvs_set_str(nvs, "url", url ? url : "");
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (++s_retries <= 8) esp_wifi_connect();
        else xEventGroupSetBits(s_wifi_events, FAILED_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_retries = 0;
        xEventGroupSetBits(s_wifi_events, CONNECTED_BIT);
    }
}

static esp_err_t connect_wifi(const char *ssid, const char *password)
{
    s_wifi_events = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_wifi_events, ESP_ERR_NO_MEM, TAG, "event group");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL), TAG, "wifi handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL), TAG, "ip handler");

    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, password, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    cfg.sta.pmf_cfg.capable = true;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &cfg), TAG, "wifi config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, CONNECTED_BIT | FAILED_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    return bits & CONNECTED_BIT ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void ota_task(void *arg)
{
    char ssid[33] = {0}, password[65] = {0}, url[256] = {0};
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("wifi_ota", NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        err = load_string(nvs, "ssid", ssid, sizeof(ssid));
        if (err == ESP_OK) err = load_string(nvs, "password", password, sizeof(password));
        if (err == ESP_OK) err = load_string(nvs, "url", url, sizeof(url));
        nvs_close(nvs);
    }
    if (err != ESP_OK || !ssid[0] || strncmp(url, "https://", 8) != 0) {
        ESP_LOGE(TAG, "Wi-Fi credentials or HTTPS firmware URL are not configured");
        goto failed;
    }

    publish(WIFI_OTA_CONNECTING, 0);
    if (connect_wifi(ssid, password) != ESP_OK) goto failed;

    publish(WIFI_OTA_DOWNLOADING, 5);
    esp_http_client_config_t http = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota = {.http_config = &http};
    esp_https_ota_handle_t handle = NULL;
    if (esp_https_ota_begin(&ota, &handle) != ESP_OK) goto failed;

    const esp_app_desc_t *running = esp_app_get_description();
    esp_app_desc_t incoming;
    if (esp_https_ota_get_img_desc(handle, &incoming) != ESP_OK ||
        strcmp(incoming.project_name, running->project_name) != 0 ||
        strcmp(incoming.version, running->version) == 0) {
        ESP_LOGE(TAG, "Rejected firmware project/version: %s/%s", incoming.project_name, incoming.version);
        esp_https_ota_abort(handle);
        goto failed;
    }

    while ((err = esp_https_ota_perform(handle)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
        int read = esp_https_ota_get_image_len_read(handle);
        uint8_t progress = (uint8_t)(read > 0 ? 10 + (read / 40000) : 10);
        publish(WIFI_OTA_DOWNLOADING, progress > 90 ? 90 : progress);
    }
    if (err != ESP_OK || !esp_https_ota_is_complete_data_received(handle)) {
        esp_https_ota_abort(handle);
        goto failed;
    }
    publish(WIFI_OTA_INSTALLING, 95);
    if (esp_https_ota_finish(handle) != ESP_OK) goto failed;
    publish(WIFI_OTA_SUCCESS, 100);
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();

failed:
    publish(WIFI_OTA_FAILED, 0);
    if (s_wifi_events) {
        esp_wifi_stop();
        vEventGroupDelete(s_wifi_events);
        s_wifi_events = NULL;
    }
    s_running = false;
    vTaskDelete(NULL);
}

esp_err_t wifi_ota_init(wifi_ota_status_cb_t callback)
{
    s_callback = callback;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGI(TAG, "New OTA image is pending verification");
        return xTaskCreate(confirm_image_task, "ota_confirm", 3072, NULL, 3, NULL) == pdPASS
            ? ESP_OK : ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t wifi_ota_start(void)
{
    if (s_running) return ESP_ERR_INVALID_STATE;
    s_running = true;
    if (xTaskCreate(ota_task, "wifi_ota", 8192, NULL, 6, NULL) != pdPASS) {
        s_running = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

const char *wifi_ota_version(void)
{
    return esp_app_get_description()->version;
}
