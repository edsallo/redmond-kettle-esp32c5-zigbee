#include "zigbee_bridge.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include "ezbee/nwk.h"
#include "ezbee/test_utils.h"
#include "ezbee/zcl/cluster/custom.h"
#include "ezbee/zcl/zcl_general_cmd.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kettle_ble.h"
#include <limits.h>
#include <string.h>

#define KETTLE_EP 1
#define MANUFACTURER_NAME "\x08" "EdS Home"
#define MODEL_IDENTIFIER  "\x0b" "RK-G211S-ZB"
#define OTA_CLUSTER_ID 0xFC00
#define OTA_ATTR_COMMAND 0x0000
#define OTA_ATTR_STATUS 0x0001
#define OTA_ATTR_PROGRESS 0x0002
#define OTA_ATTR_VERSION 0x0003
#define OTA_ATTR_WIFI_SSID 0x0010
#define OTA_ATTR_WIFI_PASSWORD 0x0011
#define OTA_ATTR_FIRMWARE_URL 0x0012

static const char *TAG = "ZB_KETTLE";
static bool s_stack_started;
static bool s_joined;
static bool s_retry_pending;
static bool s_report_task_started;
static volatile uint8_t s_tx_failures;
static volatile bool s_route_repair_pending;
static int16_t s_last_reported_temperature = INT16_MIN;
static bool s_last_reported_power;
static bool s_has_reported_power;
static uint8_t s_ota_command;
static uint8_t s_ota_status;
static uint8_t s_ota_progress;
static uint8_t s_ota_version[33];
static uint8_t s_wifi_ssid[33];
static uint8_t s_wifi_password[65];
static uint8_t s_firmware_url[255];

static void heartbeat_task(void *arg);

void zigbee_bridge_dump_network(void)
{
    if (!s_stack_started) return;
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ESP_LOGI(TAG, "NETWORK SNAPSHOT joined=%d", s_joined);
    ezb_nwk_info_iterator_t iterator = NULL;
    ezb_nwk_neighbor_info_t neighbor;
    while (ezb_nwk_get_next_neighbor(&iterator, &neighbor) == EZB_ERR_NONE) {
        ESP_LOGI(TAG, "Neighbor short=0x%04x lqi=%u rssi=%d out_cost=%u",
                 neighbor.short_addr, neighbor.lqi, neighbor.rssi, neighbor.outgoing_cost);
    }
    iterator = NULL;
    ezb_nwk_route_info_t route;
    while (ezb_nwk_get_next_route(&iterator, &route) == EZB_ERR_NONE) {
        ESP_LOGI(TAG, "Route dest=0x%04x next=0x%04x state=%u expiry=%u",
                 route.dest_addr, route.next_hop_addr, route.flags.status, route.expiry);
    }
    esp_zigbee_lock_release();
}

static void route_repair_task(void *arg)
{
    /* Drop only the stale route to the coordinator.  The next report makes
       the Zigbee stack discover a new path through any available router. */
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_nwk_route_delete(0x0000);
    const ezb_zdo_device_annce_req_t announce = {0};
    ezb_zdo_device_annce_req(&announce);
    esp_zigbee_lock_release();
    ESP_LOGW(TAG, "Coordinator route invalidated; rediscovery requested");
    s_tx_failures = 0;
    s_route_repair_pending = false;
    vTaskDelete(NULL);
}

static void schedule_route_repair(void)
{
    if (!s_route_repair_pending) {
        s_route_repair_pending = true;
        xTaskCreate(route_repair_task, "zb_route_repair", 3072, NULL, 4, NULL);
    }
}

static void secure_rejoin_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    ezb_nwk_leave_req_t request = {.rejoin = true, .remove_children = false};
    ezb_nwk_get_extended_address(&request.device_address);
    esp_zigbee_lock_acquire(portMAX_DELAY);
    const ezb_err_t err = ezb_nwk_leave_request(&request);
    esp_zigbee_lock_release();
    ESP_LOGW(TAG, "No active Zigbee links; secure rejoin requested: 0x%x", err);
    vTaskDelete(NULL);
}

static void start_heartbeat(void)
{
    if (!s_report_task_started) {
        s_report_task_started = true;
        xTaskCreate(heartbeat_task, "zb_heartbeat", 4096, NULL, 3, NULL);
    }
}

static void report_confirmation(ezb_af_user_cnf_t *confirmation, void *user_ctx)
{
    if (confirmation && confirmation->status == 0) {
        s_tx_failures = 0;
        return;
    }
    if (s_tx_failures < UINT8_MAX) s_tx_failures++;
    if (s_tx_failures >= 2) schedule_route_repair();
}

static void report_attribute_locked(uint16_t cluster, uint16_t attribute)
{
    const ezb_zcl_report_attr_cmd_t report = {
        .cmd_ctrl = {
            .dst_addr = EZB_ADDRESS_SHORT(0x0000),
            .dst_ep = 1,
            .src_ep = KETTLE_EP,
            .cluster_id = cluster,
            .manuf_code = EZB_ZCL_STD_MANUF_CODE,
            .fc = {
                .manuf_specific = 0,
                .direction = EZB_ZCL_CMD_DIRECTION_TO_CLI,
                .dis_default_rsp = 1,
            },
            .cnf_ctx = {.cb = report_confirmation, .user_ctx = NULL},
        },
        .payload.attr_id = attribute,
    };
    const ezb_err_t err = ezb_zcl_report_attr_cmd_req(&report);
    if (err != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "Attribute report 0x%04x/0x%04x queue error: 0x%x",
                 cluster, attribute, err);
    }
}

static void report_state_locked(void)
{
    report_attribute_locked(EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
                            EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID);
    report_attribute_locked(EZB_ZCL_CLUSTER_ID_ON_OFF,
                            EZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
}

static void heartbeat_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    for (;;) {
        if (s_joined && s_stack_started) {
            zigbee_bridge_dump_network();
            esp_zigbee_lock_acquire(portMAX_DELAY);
            report_state_locked();
            esp_zigbee_lock_release();
        }
        vTaskDelay(pdMS_TO_TICKS(45000));
    }
}

static void zcl_string_set(uint8_t *destination, size_t capacity, const char *source)
{
    size_t length = source ? strnlen(source, capacity - 1) : 0;
    destination[0] = (uint8_t)length;
    if (length) memcpy(destination + 1, source, length);
}

static void zcl_string_get(const uint8_t *source, char *destination, size_t capacity)
{
    size_t length = source[0];
    if (length >= capacity) length = capacity - 1;
    memcpy(destination, source + 1, length);
    destination[length] = '\0';
}

static void steering_retry_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
    esp_zigbee_lock_release();
    s_retry_pending = false;
    vTaskDelete(NULL);
}

static void set_attribute(uint16_t cluster, uint16_t attribute, void *value)
{
    if (!s_stack_started) return;
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zcl_set_attr_value(KETTLE_EP, cluster, EZB_ZCL_CLUSTER_SERVER,
                           attribute, EZB_ZCL_STD_MANUF_CODE, value, false);
    esp_zigbee_lock_release();
}

void zigbee_bridge_update(const r4s_status_t *status, bool available)
{
    if (!status || !available) return;
    int16_t temperature = (int16_t)status->temperature * 100;
    int16_t target = (int16_t)(status->target_temperature >= 40 ?
                               status->target_temperature : 100) * 100;
    bool power = status->is_on;
    uint8_t system_mode = power ? EZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT :
                                  EZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;

    set_attribute(EZB_ZCL_CLUSTER_ID_ON_OFF,
                  EZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &power);
    set_attribute(EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
                  EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
                  &temperature);
    set_attribute(EZB_ZCL_CLUSTER_ID_THERMOSTAT,
                  EZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, &temperature);
    set_attribute(EZB_ZCL_CLUSTER_ID_THERMOSTAT,
                  EZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &target);
    set_attribute(EZB_ZCL_CLUSTER_ID_THERMOSTAT,
                  EZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, &system_mode);

    const bool temperature_changed = temperature != s_last_reported_temperature;
    const bool power_changed = !s_has_reported_power || power != s_last_reported_power;
    if (s_joined && (temperature_changed || power_changed)) {
        esp_zigbee_lock_acquire(portMAX_DELAY);
        if (temperature_changed) {
            report_attribute_locked(EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
                                    EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID);
            s_last_reported_temperature = temperature;
        }
        if (power_changed) {
            report_attribute_locked(EZB_ZCL_CLUSTER_ID_ON_OFF,
                                    EZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
            s_last_reported_power = power;
            s_has_reported_power = true;
        }
        esp_zigbee_lock_release();
    }
}

void zigbee_bridge_update_ota_status(wifi_ota_status_t status, uint8_t progress)
{
    s_ota_status = (uint8_t)status;
    s_ota_progress = progress;
    set_attribute(OTA_CLUSTER_ID, OTA_ATTR_STATUS, &s_ota_status);
    set_attribute(OTA_CLUSTER_ID, OTA_ATTR_PROGRESS, &s_ota_progress);
}

static void set_attr_handler(ezb_zcl_set_attr_value_message_t *message)
{
    if (!message || message->info.status != EZB_ZCL_STATUS_SUCCESS) return;

    const uint16_t attr = message->in.attribute.id;
    void *value = message->in.attribute.data.value;
    esp_err_t result = ESP_OK;

    if (message->info.cluster_id == EZB_ZCL_CLUSTER_ID_ON_OFF &&
        attr == EZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
        result = kettle_ble_set_power(*(bool *)value);
    } else if (message->info.cluster_id == EZB_ZCL_CLUSTER_ID_THERMOSTAT &&
               attr == EZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
        int target = (*(int16_t *)value + 50) / 100;
        if (target < 40) target = 40;
        if (target > 100) target = 100;
        result = kettle_ble_heat_to((uint8_t)target);
    } else if (message->info.cluster_id == EZB_ZCL_CLUSTER_ID_THERMOSTAT &&
               attr == EZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID) {
        const uint8_t mode = *(uint8_t *)value;
        result = mode == EZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF ?
                 kettle_ble_set_power(false) : ESP_OK;
    } else if (message->info.cluster_id == OTA_CLUSTER_ID) {
        if (attr == OTA_ATTR_COMMAND && *(uint8_t *)value == 1) {
            result = wifi_ota_start();
            s_ota_command = 0;
            set_attribute(OTA_CLUSTER_ID, OTA_ATTR_COMMAND, &s_ota_command);
        } else if (attr == OTA_ATTR_WIFI_SSID || attr == OTA_ATTR_WIFI_PASSWORD ||
                   attr == OTA_ATTR_FIRMWARE_URL) {
            uint8_t *destination = attr == OTA_ATTR_WIFI_SSID ? s_wifi_ssid :
                                   attr == OTA_ATTR_WIFI_PASSWORD ? s_wifi_password :
                                   s_firmware_url;
            size_t capacity = attr == OTA_ATTR_WIFI_SSID ? sizeof(s_wifi_ssid) :
                              attr == OTA_ATTR_WIFI_PASSWORD ? sizeof(s_wifi_password) :
                              sizeof(s_firmware_url);
            size_t encoded_length = (size_t)((uint8_t *)value)[0] + 1;
            if (encoded_length > capacity) encoded_length = capacity;
            memcpy(destination, value, encoded_length);
            char ssid[33], password[65], url[255];
            zcl_string_get(s_wifi_ssid, ssid, sizeof(ssid));
            zcl_string_get(s_wifi_password, password, sizeof(password));
            zcl_string_get(s_firmware_url, url, sizeof(url));
            result = wifi_ota_save_config(ssid, password, url);
        }
    }

    message->out.result = result == ESP_OK ? EZB_ZCL_STATUS_SUCCESS :
                                             EZB_ZCL_STATUS_FAIL;
}

static void core_action_handler(ezb_zcl_core_action_callback_id_t id, void *message)
{
    if (id == EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID) set_attr_handler(message);
}

static bool signal_handler(const ezb_app_signal_t *signal)
{
    const ezb_app_signal_type_t type = ezb_app_signal_get_type(signal);
    switch (type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        const ezb_bdb_comm_status_t status =
            *(ezb_bdb_comm_status_t *)ezb_app_signal_get_params(signal);
        if (status == EZB_BDB_STATUS_SUCCESS) {
            s_joined = !ezb_bdb_is_factory_new();
            if (s_joined) start_heartbeat();
            if (ezb_bdb_is_factory_new()) {
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
            }
        } else {
            ESP_LOGW(TAG, "Zigbee initialization failed: 0x%02x", status);
        }
        break;
    }
    case EZB_BDB_SIGNAL_STEERING: {
        const ezb_bdb_comm_status_t status =
            *(ezb_bdb_comm_status_t *)ezb_app_signal_get_params(signal);
        ESP_LOGI(TAG, "Network steering: %s",
                 status == EZB_BDB_STATUS_SUCCESS ? "joined" : "failed");
        s_joined = status == EZB_BDB_STATUS_SUCCESS;
        if (s_joined) start_heartbeat();
        if (!s_joined && !s_retry_pending) {
            s_retry_pending = true;
            xTaskCreate(steering_retry_task, "zb_retry", 3072, NULL, 4, NULL);
        }
        break;
    }
    case EZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        ESP_LOGW(TAG, "Zigbee has no active links");
        xTaskCreate(secure_rejoin_task, "zb_rejoin", 3072, NULL, 4, NULL);
        break;
    case EZB_NWK_SIGNAL_NETWORK_STATUS: {
        const ezb_nwk_signal_network_status_params_t *params =
            (const ezb_nwk_signal_network_status_params_t *)ezb_app_signal_get_params(signal);
        const uint8_t status = params ? params->status : 0xff;
        ESP_LOGW(TAG, "Zigbee network status 0x%02x for 0x%04x", status,
                 params ? params->network_addr : 0xffff);
        if (status == EZB_NWK_NETWORK_STATUS_LEGACY_NO_ROUTE_AVAILABLE ||
            status == EZB_NWK_NETWORK_STATUS_LEGACY_LINK_FAILURE ||
            status == EZB_NWK_NETWORK_STATUS_LINK_FAILURE ||
            status == EZB_NWK_NETWORK_STATUS_TARGET_DEVICE_UNAVAILABLE ||
            status == EZB_NWK_NETWORK_STATUS_SOURCE_ROUTE_FAILURE ||
            status == EZB_NWK_NETWORK_STATUS_MANY_TO_ONE_ROUTE_FAILURE) {
            schedule_route_repair();
        }
        break;
    }
    case EZB_ZDO_SIGNAL_LEAVE:
    case EZB_ZDO_SIGNAL_LEAVE_INDICATION:
        s_joined = false;
        ESP_LOGW(TAG, "Device left Zigbee network");
        break;
    default:
        break;
    }
    return true;
}

static esp_err_t register_device(void)
{
    ezb_af_device_desc_t device = ezb_af_create_device_desc();
    ezb_zha_thermostat_config_t cfg = EZB_ZHA_THERMOSTAT_CONFIG();
    cfg.thermostat_cfg.local_temperature = 2000;
    cfg.thermostat_cfg.control_sequence_of_operation =
        EZB_ZCL_THERMOSTAT_CONTROL_SEQUENCE_OF_OPERATION_HEATING_ONLY;
    cfg.thermostat_cfg.system_mode = EZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;

    ezb_af_ep_desc_t ep = ezb_zha_create_thermostat(KETTLE_EP, &cfg);
    ezb_zcl_cluster_desc_t basic = ezb_af_endpoint_get_cluster_desc(
        ep, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(
        basic, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME);
    ezb_zcl_basic_cluster_desc_add_attr(
        basic, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER);

    int16_t target = 10000;
    ezb_zcl_cluster_desc_t thermostat = ezb_af_endpoint_get_cluster_desc(
        ep, EZB_ZCL_CLUSTER_ID_THERMOSTAT, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_thermostat_cluster_desc_add_attr(
        thermostat, EZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &target);

    ezb_zcl_on_off_cluster_server_config_t onoff_cfg = {.on_off = false};
    ESP_RETURN_ON_ERROR(ezb_af_endpoint_add_cluster_desc(
        ep, ezb_zcl_on_off_create_cluster_desc(&onoff_cfg, EZB_ZCL_CLUSTER_SERVER)),
        TAG, "add OnOff cluster");

    ezb_zcl_temperature_measurement_cluster_server_config_t temp_cfg = {
        .measured_value = 2000,
        .min_measured_value = 0,
        .max_measured_value = 10000,
    };
    ESP_RETURN_ON_ERROR(ezb_af_endpoint_add_cluster_desc(
        ep, ezb_zcl_temperature_measurement_create_cluster_desc(
                &temp_cfg, EZB_ZCL_CLUSTER_SERVER)), TAG, "add temperature cluster");

    zcl_string_set(s_ota_version, sizeof(s_ota_version), wifi_ota_version());
    ezb_zcl_custom_cluster_config_t ota_cfg = {.cluster_id = OTA_CLUSTER_ID};
    ezb_zcl_cluster_desc_t ota_cluster = ezb_zcl_custom_create_cluster_desc(
        &ota_cfg, EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(ota_cluster, ESP_FAIL, TAG, "create OTA cluster");
    const uint8_t rw = EZB_ZCL_ATTR_ACCESS_READ_WRITE;
    const uint8_t ro_report = EZB_ZCL_ATTR_ACCESS_READ | EZB_ZCL_ATTR_ACCESS_REPORTING;
    ESP_RETURN_ON_ERROR(ezb_zcl_custom_cluster_desc_add_attr(ota_cluster, OTA_ATTR_COMMAND,
        EZB_ZCL_ATTR_TYPE_UINT8, rw, &s_ota_command), TAG, "OTA command attr");
    ESP_RETURN_ON_ERROR(ezb_zcl_custom_cluster_desc_add_attr(ota_cluster, OTA_ATTR_STATUS,
        EZB_ZCL_ATTR_TYPE_ENUM8, ro_report, &s_ota_status), TAG, "OTA status attr");
    ESP_RETURN_ON_ERROR(ezb_zcl_custom_cluster_desc_add_attr(ota_cluster, OTA_ATTR_PROGRESS,
        EZB_ZCL_ATTR_TYPE_UINT8, ro_report, &s_ota_progress), TAG, "OTA progress attr");
    ESP_RETURN_ON_ERROR(ezb_zcl_custom_cluster_desc_add_attr(ota_cluster, OTA_ATTR_VERSION,
        EZB_ZCL_ATTR_TYPE_STRING, EZB_ZCL_ATTR_ACCESS_READ, s_ota_version), TAG, "OTA version attr");
    ESP_RETURN_ON_ERROR(ezb_zcl_custom_cluster_desc_add_attr(ota_cluster, OTA_ATTR_WIFI_SSID,
        EZB_ZCL_ATTR_TYPE_STRING, rw, s_wifi_ssid), TAG, "Wi-Fi SSID attr");
    ESP_RETURN_ON_ERROR(ezb_zcl_custom_cluster_desc_add_attr(ota_cluster, OTA_ATTR_WIFI_PASSWORD,
        EZB_ZCL_ATTR_TYPE_STRING, EZB_ZCL_ATTR_ACCESS_WRITE, s_wifi_password), TAG, "Wi-Fi password attr");
    ESP_RETURN_ON_ERROR(ezb_zcl_custom_cluster_desc_add_attr(ota_cluster, OTA_ATTR_FIRMWARE_URL,
        EZB_ZCL_ATTR_TYPE_STRING, rw, s_firmware_url), TAG, "firmware URL attr");
    ESP_RETURN_ON_ERROR(ezb_af_endpoint_add_cluster_desc(ep, ota_cluster), TAG, "add OTA cluster");

    ESP_RETURN_ON_ERROR(ezb_af_device_add_endpoint_desc(device, ep), TAG, "add endpoint");
    ESP_RETURN_ON_ERROR(ezb_af_device_desc_register(device), TAG, "register device");
    ezb_zcl_core_action_handler_register(core_action_handler);
    return ESP_OK;
}

static void zigbee_task(void *arg)
{
    esp_zigbee_config_t config = {
        .device_config = {
            .device_type = EZB_NWK_DEVICE_TYPE_ROUTER,
            .install_code_policy = false,
            .zczr_config = {.max_children = 0},
        },
        .platform_config = {
            .storage_partition_name = "nvs",
            .radio_config = {.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE},
        },
    };
    ESP_ERROR_CHECK(esp_zigbee_init(&config));
    ezb_set_tx_power(20);
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(0x07FFF800UL));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(signal_handler));
    ESP_ERROR_CHECK(register_device());
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    s_stack_started = true;
    esp_zigbee_launch_mainloop();
    vTaskDelete(NULL);
}

esp_err_t zigbee_bridge_start(void)
{
    return xTaskCreate(zigbee_task, "zigbee", 6144, NULL, 5, NULL) == pdPASS ?
           ESP_OK : ESP_ERR_NO_MEM;
}

bool zigbee_bridge_is_joined(void) { return s_joined; }

void zigbee_bridge_factory_reset(void)
{
    if (!s_stack_started) return;
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_bdb_reset_via_local_action();
    esp_zigbee_lock_release();
    s_joined = false;
}
