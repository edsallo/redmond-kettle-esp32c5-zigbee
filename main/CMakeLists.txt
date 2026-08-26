idf_component_register(
  SRCS "app_main.c" "redmond_protocol.c" "kettle_ble.c" "zigbee_bridge.c" "status_panel.c"
  INCLUDE_DIRS "."
  REQUIRES bt nvs_flash esp_timer esp_driver_gpio
)
