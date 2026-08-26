#include "zigbee_bridge.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_zigbee.h"
#include "ezbee/zha.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kettle_ble.h"

#define KETTLE_EP 1
#define MANUFACTURER_NAME "\x08" "EdS Home"
#define MODEL_IDENTIFIER  "\x0b" "RK-G211S-ZB"

static const char *TAG = "ZB_KETTLE";
static bool s_stack_started;
static bool s_joined;
static bool s_retry_pending;

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
        if (!s_joined && !s_retry_pending) {
            s_retry_pending = true;
            xTaskCreate(steering_retry_task, "zb_retry", 3072, NULL, 4, NULL);
        }
        break;
    }
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
