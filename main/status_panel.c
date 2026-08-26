#include "status_panel.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kettle_ble.h"
#include "led_strip.h"
#include "zigbee_bridge.h"

#define SHARED_BOOT_RGB_GPIO GPIO_NUM_27

static const char *TAG = "STATUS_PANEL";

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb_t;

static void show_color(rgb_t color)
{
    /* GPIO27 is shared by BOOT and the WS2812. Attach RMT only while sending
       a pixel, then return the pin to a pulled-up input for safe button reads. */
    led_strip_handle_t strip = NULL;
    led_strip_config_t strip_config = {
        .strip_gpio_num = SHARED_BOOT_RGB_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    if (led_strip_new_rmt_device(&strip_config, &rmt_config, &strip) == ESP_OK) {
        led_strip_set_pixel(strip, 0, color.red, color.green, color.blue);
        led_strip_refresh(strip);
        led_strip_del(strip);
    }
    gpio_reset_pin(SHARED_BOOT_RGB_GPIO);
    gpio_set_direction(SHARED_BOOT_RGB_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SHARED_BOOT_RGB_GPIO, GPIO_PULLUP_ONLY);
}

static void panel_task(void *arg)
{
    const rgb_t off = {0, 0, 0};
    /* Keep the board indicator visible without lighting the room: roughly
       10% of the original per-channel levels. */
    const rgb_t green = {0, 2, 0};
    const rgb_t yellow = {2, 1, 0};
    const rgb_t purple = {1, 0, 2};
    const rgb_t red = {2, 0, 0};
    bool phase = false;
    bool pressed = false;
    TickType_t pressed_at = 0;

    gpio_reset_pin(SHARED_BOOT_RGB_GPIO);
    gpio_set_direction(SHARED_BOOT_RGB_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SHARED_BOOT_RGB_GPIO, GPIO_PULLUP_ONLY);

    for (;;) {
        const bool down = gpio_get_level(SHARED_BOOT_RGB_GPIO) == 0;
        const TickType_t now = xTaskGetTickCount();

        if (down && !pressed) {
            pressed = true;
            pressed_at = now;
        } else if (!down && pressed) {
            const uint32_t held_ms = pdTICKS_TO_MS(now - pressed_at);
            pressed = false;
            if (held_ms >= 10000) {
                ESP_LOGW(TAG, "BOOT 10 s: factory-reset Zigbee network");
                show_color(red);
                zigbee_bridge_factory_reset();
            } else if (held_ms >= 5000) {
                ESP_LOGW(TAG, "BOOT 5 s: forget kettle and restart discovery");
                kettle_ble_forget_and_reconnect();
            } else if (held_ms >= 50) {
                ESP_LOGI(TAG, "BOOT: restart kettle discovery");
                kettle_ble_reconnect();
            }
        }

        phase = !phase;
        if (pressed) {
            /* Never drive the shared line while the physical button grounds it. */
        } else if (!zigbee_bridge_is_joined()) {
            show_color(phase ? purple : off);
        } else if (!kettle_ble_is_connected()) {
            show_color(yellow);
        } else {
            show_color(green);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t status_panel_start(void)
{
    return xTaskCreate(panel_task, "status_panel", 4096, NULL, 4, NULL) == pdPASS ?
           ESP_OK : ESP_ERR_NO_MEM;
}
