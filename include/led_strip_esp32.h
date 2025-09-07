// The ESP-IDF framework to control the on-board RGB LED on the
// ESP32-S3 DevKitC. The on-board LED is a WS2812B (NeoPixel) addressable LED,
// typically connected to GPIO48. This code uses the `led_strip` driver to
// achieve full color control.
#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "led_strip.h"

static const char *TAG = "LED_STATUS";

// Define the GPIO pin for the on-board RGB LED
#define RGB_LED_GPIO_PIN GPIO_NUM_48

// Define the number of LEDs in the strip (1 for the on-board LED)
#define RGB_LED_STRIP_LENGTH 1

// Declare a handle for the LED strip
led_strip_handle_t led_strip_handle;

// Function to initialize the LED strip using the RMT peripheral
static void led_strip_init() {
    ESP_LOGD(TAG, "Initializing LED strip...");

    // Configure the LED strip driver
    led_strip_config_t led_strip_config = {
        .strip_gpio_num = RGB_LED_GPIO_PIN,
        .max_leds = RGB_LED_STRIP_LENGTH,
        .led_model = LED_MODEL_WS2812,
    };

    // Configure the RMT peripheral for the LED strip
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, // 10MHz resolution for NeoPixel timing
        .flags = {
            .with_dma = false, // Same as above
        },
    };

    // Initialize the LED strip and get the handle
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_config, &rmt_config, &led_strip_handle));
    ESP_LOGD(TAG, "LED strip initialized.");
}

void set_led_color(uint32_t red, uint32_t green, uint32_t blue) {
    ESP_LOGD(TAG, "Setting LED color to R:%d G:%d B:%d", red, green, blue);
    // Set the color of the LED
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip_handle, 0, red, green, blue));
    // Refresh to apply the color change
    ESP_ERROR_CHECK(led_strip_refresh(led_strip_handle));
}

void blink_rgb(int red, int green, int blue, int delay_ms) {
    set_led_color(red, green, blue);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    set_led_color(0, 0, 0); // Turn off the LED
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}