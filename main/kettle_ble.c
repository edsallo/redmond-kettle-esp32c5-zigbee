#include "kettle_ble.h"

#include <string.h>
#include <time.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define EVT_READY    BIT0
#define EVT_RESPONSE BIT1
#define EVT_DISCONNECTED BIT2

typedef enum { REQUEST_POWER, REQUEST_TEMPERATURE } request_type_t;
typedef struct { request_type_t type; uint8_t value; } control_request_t;

static const char *TAG = "BLE_KETTLE";
static const ble_uuid128_t SERVICE_UUID = BLE_UUID128_INIT(
    0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x01,0x00,0x40,0x6e);
static const ble_uuid128_t RX_UUID = BLE_UUID128_INIT(
    0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x02,0x00,0x40,0x6e);
static const ble_uuid128_t TX_UUID = BLE_UUID128_INIT(
    0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x03,0x00,0x40,0x6e);

static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_service_start, s_service_end, s_rx_handle, s_tx_handle;
static uint8_t s_counter, s_wait_counter, s_wait_command;
static uint8_t s_response[32];
static size_t s_response_len;
static bool s_connected;
static EventGroupHandle_t s_events;
static SemaphoreHandle_t s_command_lock;
static QueueHandle_t s_control_queue;
static kettle_status_callback_t s_status_callback;
static ble_addr_t s_bound_addr;
static bool s_has_bound_addr;

static void start_scan(void);

static bool advertised_name_matches(const struct ble_hs_adv_fields *fields)
{
    static const char expected[] = "RK-G211S";
    return fields->name && fields->name_len == sizeof(expected) - 1 &&
           memcmp(fields->name, expected, sizeof(expected) - 1) == 0;
}

static void load_bound_addr(void)
{
    nvs_handle_t nvs;
    size_t length = sizeof(s_bound_addr);
    if (nvs_open("kettle", NVS_READONLY, &nvs) == ESP_OK) {
        s_has_bound_addr = nvs_get_blob(nvs, "ble_addr", &s_bound_addr, &length) == ESP_OK &&
                           length == sizeof(s_bound_addr);
        nvs_close(nvs);
    }
}

static void save_bound_addr(const ble_addr_t *address)
{
    nvs_handle_t nvs;
    if (nvs_open("kettle", NVS_READWRITE, &nvs) == ESP_OK) {
        if (nvs_set_blob(nvs, "ble_addr", address, sizeof(*address)) == ESP_OK &&
            nvs_commit(nvs) == ESP_OK) {
            s_bound_addr = *address;
            s_has_bound_addr = true;
        }
        nvs_close(nvs);
    }
}

static int characteristic_cb(uint16_t conn, const struct ble_gatt_error *error,
                             const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == 0 && chr) {
        if (ble_uuid_cmp(&chr->uuid.u, &RX_UUID.u) == 0) s_rx_handle = chr->val_handle;
        if (ble_uuid_cmp(&chr->uuid.u, &TX_UUID.u) == 0) s_tx_handle = chr->val_handle;
        return 0;
    }
    if (error->status == BLE_HS_EDONE && s_rx_handle && s_tx_handle) {
        /* The CCCD immediately follows TX on the RK-G211S UART service. */
        uint16_t notify = 1;
        int rc = ble_gattc_write_flat(conn, s_tx_handle + 1, &notify, sizeof(notify), NULL, NULL);
        if (rc == 0) xEventGroupSetBits(s_events, EVT_READY);
    }
    return 0;
}

static int service_cb(uint16_t conn, const struct ble_gatt_error *error,
                      const struct ble_gatt_svc *service, void *arg)
{
    if (error->status == 0 && service) {
        s_service_start = service->start_handle;
        s_service_end = service->end_handle;
        return 0;
    }
    if (error->status == BLE_HS_EDONE && s_service_start) {
        ble_gattc_disc_all_chrs(conn, s_service_start, s_service_end,
                               characteristic_cb, NULL);
    }
    return 0;
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                    event->disc.length_data) != 0 ||
            !advertised_name_matches(&fields)) return 0;
        if (s_has_bound_addr && ble_addr_cmp(&s_bound_addr, &event->disc.addr) != 0) return 0;
        if (!s_has_bound_addr) save_bound_addr(&event->disc.addr);
        ble_gap_disc_cancel();
        struct ble_gap_conn_params params;
        memset(&params, 0, sizeof(params));
        params.scan_itvl = 0x0010;
        params.scan_window = 0x0010;
        params.itvl_min = 24;
        params.itvl_max = 40;
        params.latency = 0;
        /* The kettle loses power when lifted from the base. Detect that event
         * promptly while retaining ample margin for the 30-50 ms interval. */
        params.supervision_timeout = 150;
        params.min_ce_len = 0;
        params.max_ce_len = 0;
        ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 10000,
                        &params, gap_event, NULL);
        break;
    }
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            xEventGroupClearBits(s_events, EVT_DISCONNECTED);
            s_conn = event->connect.conn_handle;
            s_rx_handle = s_tx_handle = s_service_start = s_service_end = 0;
            ble_gattc_disc_svc_by_uuid(s_conn, &SERVICE_UUID.u, service_cb, NULL);
        } else {
            start_scan();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        s_connected = false;
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        xEventGroupClearBits(s_events, EVT_READY | EVT_RESPONSE);
        xEventGroupSetBits(s_events, EVT_DISCONNECTED);
        if (s_status_callback) s_status_callback(NULL, false);
        start_scan();
        break;
    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint8_t packet[40];
        const uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len > sizeof(packet)) break;
        os_mbuf_copydata(event->notify_rx.om, 0, len, packet);
        if (len >= 4 && packet[0] == 0x55 && packet[1] == s_wait_counter &&
            packet[2] == s_wait_command) {
            s_response_len = len - 4;
            memcpy(s_response, packet + 3, s_response_len);
            xEventGroupSetBits(s_events, EVT_RESPONSE);
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

static void start_scan(void)
{
    if (ble_gap_disc_active()) return;
    struct ble_gap_disc_params params = {
        .itvl = 0,
        .window = 0,
        .filter_policy = BLE_HCI_SCAN_FILT_NO_WL,
        .limited = 0,
        .passive = 0,
        .filter_duplicates = 1,
    };
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params, gap_event, NULL);
}

static esp_err_t command(uint8_t id, const uint8_t *payload, size_t payload_len,
                         uint8_t *response, size_t *response_len)
{
    /* Authentication is the first protocol command, before s_connected is set. */
    if (s_conn == BLE_HS_CONN_HANDLE_NONE || !s_rx_handle) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_command_lock, pdMS_TO_TICKS(5000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    uint8_t packet[32];
    const size_t packet_len = r4s_make_packet(s_counter, id, payload,
                                               payload_len, packet, sizeof(packet));
    s_wait_counter = s_counter;
    s_wait_command = id;
    s_response_len = 0;
    xEventGroupClearBits(s_events, EVT_RESPONSE);
    int rc = ble_gattc_write_flat(s_conn, s_rx_handle, packet, packet_len, NULL, NULL);
    esp_err_t result = ESP_FAIL;
    EventBits_t result_bits = 0;
    if (rc == 0) {
        /* Authentication normally answers within a few hundred milliseconds.
         * Directly after a power cycle the kettle can advertise before its R4S
         * protocol is ready; fail this probe quickly and retry the connection. */
        const TickType_t timeout = id == R4S_CMD_AUTH
            ? pdMS_TO_TICKS(800)
            : pdMS_TO_TICKS(4500);
        result_bits = xEventGroupWaitBits(s_events, EVT_RESPONSE | EVT_DISCONNECTED,
                                           pdTRUE, pdFALSE, timeout);
    }
    if (result_bits & EVT_RESPONSE) {
        if (response && response_len) {
            size_t copy = *response_len < s_response_len ? *response_len : s_response_len;
            memcpy(response, s_response, copy);
            *response_len = copy;
        }
        s_counter++;
        result = ESP_OK;
    }
    xSemaphoreGive(s_command_lock);
    return result;
}

static esp_err_t authenticate(void)
{
    static const uint8_t key[8] = {'H','O','M','E','Y','2','1','1'};
    uint8_t rsp[4]; size_t len = sizeof(rsp);
    esp_err_t err = command(R4S_CMD_AUTH, key, sizeof(key), rsp, &len);
    return err == ESP_OK && len && rsp[0] ? ESP_OK : ESP_FAIL;
}

static esp_err_t set_power_now(bool on)
{
    uint8_t rsp[4]; size_t len = sizeof(rsp);
    if (on) {
        uint8_t mode[16];
        r4s_make_mode_payload(false, 0, mode);
        ESP_RETURN_ON_ERROR(command(R4S_CMD_SET_MODE, mode, sizeof(mode), rsp, &len), TAG, "boil mode");
    }
    len = sizeof(rsp);
    return command(on ? R4S_CMD_ON : R4S_CMD_OFF, NULL, 0, rsp, &len);
}

static esp_err_t heat_to_now(uint8_t target)
{
    uint8_t mode[16], rsp[4]; size_t len = sizeof(rsp);

    /* RK-G211S ignores SET_MODE while another heating program is active.
     * Stop the current program first; waiting for the command response also
     * gives the kettle enough time to commit the OFF state before SET_MODE. */
    ESP_RETURN_ON_ERROR(command(R4S_CMD_OFF, NULL, 0, rsp, &len), TAG,
                        "stop before changing temperature");

    r4s_make_mode_payload(true, target, mode);
    len = sizeof(rsp);
    ESP_RETURN_ON_ERROR(command(R4S_CMD_SET_MODE, mode, sizeof(mode), rsp, &len), TAG, "temperature mode");
    len = sizeof(rsp);
    return command(R4S_CMD_ON, NULL, 0, rsp, &len);
}

static esp_err_t restore_autonomous_backlight(void)
{
    uint8_t rsp[24];
    size_t len = sizeof(rsp);
    r4s_status_t status;
    ESP_RETURN_ON_ERROR(command(R4S_CMD_STATUS, NULL, 0, rsp, &len), TAG, "read before backlight");
    if (!r4s_decode_status(rsp, len, &status)) return ESP_ERR_INVALID_RESPONSE;

    /* Do not disturb an active temperature program after a reconnect. */
    if (!status.is_on) {
        uint8_t mode[16];
        r4s_make_mode_payload(false, 0, mode);
        len = sizeof(rsp);
        ESP_RETURN_ON_ERROR(command(R4S_CMD_SET_MODE, mode, sizeof(mode), rsp, &len),
                            TAG, "idle mode before backlight");
    }

    /* Half of the stock 0xc8 idle-pulse intensity. Heating colours remain
     * controlled autonomously by the kettle and are not changed here. */
    const uint8_t autonomous[] = {0x64, 0x64, 0x01};
    len = sizeof(rsp);
    ESP_RETURN_ON_ERROR(command(R4S_CMD_SET_SWITCH, autonomous, sizeof(autonomous), rsp, &len),
                        TAG, "autonomous backlight");
    len = sizeof(rsp);
    ESP_RETURN_ON_ERROR(command(R4S_CMD_COMMIT, NULL, 0, rsp, &len), TAG, "commit backlight");

    /* The kettle only starts its autonomous idle animation after its clock has
     * been initialised.  ESP does not have network time here, so retain a sane
     * build-time fallback; exact seconds are irrelevant to this animation. */
    int32_t epoch = (int32_t)time(NULL);
    if (epoch < 1700000000) epoch = 1787599470;
    const int32_t timezone_offset = 3 * 60 * 60;
    uint8_t clock_payload[8];
    for (size_t i = 0; i < 4; i++) {
        clock_payload[i] = (uint8_t)((uint32_t)epoch >> (i * 8));
        clock_payload[i + 4] = (uint8_t)((uint32_t)timezone_offset >> (i * 8));
    }
    len = sizeof(rsp);
    ESP_RETURN_ON_ERROR(command(R4S_CMD_TIME_SYNC, clock_payload, sizeof(clock_payload), rsp, &len),
                        TAG, "time sync for backlight");
    ESP_LOGI(TAG, "Autonomous temperature backlight restored");
    return ESP_OK;
}

static void control_task(void *arg)
{
    for (;;) {
        xEventGroupWaitBits(s_events, EVT_READY, pdFALSE, pdTRUE, portMAX_DELAY);
        esp_err_t auth_result = ESP_FAIL;
        for (int attempt = 0; attempt < 3 && s_conn != BLE_HS_CONN_HANDLE_NONE; attempt++) {
            auth_result = authenticate();
            if (auth_result == ESP_OK) break;
            ESP_LOGW(TAG, "Authorization probe %d failed", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (auth_result != ESP_OK) {
            ESP_LOGW(TAG, "Authorization failed");
            if (s_conn != BLE_HS_CONN_HANDLE_NONE) {
                ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        s_connected = true;
        ESP_LOGI(TAG, "RK-G211S connected and authorized");
        if (restore_autonomous_backlight() != ESP_OK) {
            ESP_LOGW(TAG, "Could not restore autonomous backlight");
        }

        TickType_t last_poll = 0;
        while (s_connected) {
            control_request_t request;
            if (xQueueReceive(s_control_queue, &request, pdMS_TO_TICKS(250)) == pdTRUE) {
                if (request.type == REQUEST_POWER) set_power_now(request.value != 0);
                else heat_to_now(request.value);
            }
            if (xTaskGetTickCount() - last_poll >= pdMS_TO_TICKS(2000)) {
                uint8_t rsp[24]; size_t len = sizeof(rsp);
                r4s_status_t status;
                if (command(R4S_CMD_STATUS, NULL, 0, rsp, &len) == ESP_OK &&
                    r4s_decode_status(rsp, len, &status)) {
                    ESP_LOGI(TAG, "STATUS temp=%u target=%u on=%u mode=%u error=%u",
                             status.temperature, status.target_temperature,
                             status.is_on, status.mode, status.error);
                    if (s_status_callback) s_status_callback(&status, true);
                } else {
                    /* A stale status request may finish after the kettle has
                     * already reappeared. Never terminate that new session. */
                    if (s_connected && s_conn != BLE_HS_CONN_HANDLE_NONE) {
                        ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
                    }
                }
                last_poll = xTaskGetTickCount();
            }
        }
    }
}

static void host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void on_sync(void)
{
    uint8_t address_type;
    if (ble_hs_id_infer_auto(0, &address_type) == 0) start_scan();
}

esp_err_t kettle_ble_start(kettle_status_callback_t callback)
{
    s_status_callback = callback;
    load_bound_addr();
    s_events = xEventGroupCreate();
    s_command_lock = xSemaphoreCreateMutex();
    s_control_queue = xQueueCreate(8, sizeof(control_request_t));
    if (!s_events || !s_command_lock || !s_control_queue) return ESP_ERR_NO_MEM;

    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(host_task);
    return xTaskCreate(control_task, "kettle", 6144, NULL, 6, NULL) == pdPASS ?
           ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t kettle_ble_set_power(bool on)
{
    control_request_t request = {.type = REQUEST_POWER, .value = on ? 1 : 0};
    return xQueueSend(s_control_queue, &request, pdMS_TO_TICKS(100)) == pdTRUE ?
           ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t kettle_ble_heat_to(uint8_t temperature)
{
    if (temperature < 40 || temperature > 100) return ESP_ERR_INVALID_ARG;
    control_request_t request = {.type = REQUEST_TEMPERATURE, .value = temperature};
    return xQueueSend(s_control_queue, &request, pdMS_TO_TICKS(100)) == pdTRUE ?
           ESP_OK : ESP_ERR_TIMEOUT;
}

bool kettle_ble_is_connected(void) { return s_connected; }

void kettle_ble_reconnect(void)
{
    if (s_conn != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
    } else {
        start_scan();
    }
}

void kettle_ble_forget_and_reconnect(void)
{
    nvs_handle_t nvs;
    if (nvs_open("kettle", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_key(nvs, "ble_addr");
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    s_has_bound_addr = false;
    kettle_ble_reconnect();
}
