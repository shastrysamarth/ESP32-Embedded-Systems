#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/temperature_sensor.h"

static const char *TAG = "LAB6_ULTRASONIC";

// === Pin definitions ===
#define TRIG_GPIO   2
#define ECHO_GPIO   3

// How many samples per reading (for averaging)
#define NUM_SAMPLES 5

// Timeout in microseconds for waiting on echo
#define ECHO_TIMEOUT_US 300000  // 30 ms ~ about 5 meters max

// Optional calibration offset (PCB -> sensor face), tweak during lab
#define PCB_OFFSET_CM   0.0f   // start with 0, tune if you're consistently off

// --- Helper: init GPIO for ultrasonic ---
static void ultrasonic_gpio_init(void)
{
    // TRIG as output
    gpio_config_t trig_conf = {
        .pin_bit_mask = 1ULL << TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&trig_conf);
    gpio_set_level(TRIG_GPIO, 0);

    // ECHO as input
    gpio_config_t echo_conf = {
        .pin_bit_mask = 1ULL << ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&echo_conf);
}

// --- Helper: trigger one ultrasonic pulse and return echo time in microseconds ---
static int64_t ultrasonic_pulse_read_us(void)
{
    // Make sure trig is LOW for a bit
    gpio_set_level(TRIG_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(2));

    // Debug: read echo before triggering
    int echo_before = gpio_get_level(ECHO_GPIO);

    // 10us HIGH pulse on TRIG
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);

    // Wait for ECHO to go LOW first (in case it's still high from a previous measurement)
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if (esp_timer_get_time() - t0 > ECHO_TIMEOUT_US) {
            ESP_LOGW(TAG, "Timeout waiting for ECHO to go LOW (echo_before=%d)", echo_before);
            return -1;
        }
    }

    // Now wait for ECHO rising edge (goes HIGH)
    int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if (esp_timer_get_time() - start_wait > ECHO_TIMEOUT_US) {
            ESP_LOGW(TAG, "Timeout waiting for ECHO to go HIGH (echo_before=%d)", echo_before);
            return -1;  // timeout waiting for echo start
        }
    }

    // Now ECHO is HIGH, measure how long
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if (esp_timer_get_time() - echo_start > ECHO_TIMEOUT_US) {
            ESP_LOGW(TAG, "Timeout waiting for ECHO to go LOW after HIGH");
            return -1;  // timeout waiting for echo end
        }
    }
    int64_t echo_end = esp_timer_get_time();

    int64_t duration = echo_end - echo_start;
    ESP_LOGI(TAG, "Raw echo duration_us=%lld", duration);

    return duration;  // duration in microseconds
}


// --- Helper: get ESP32-C3 internal temperature in °C ---
static float read_temperature_celsius(void)
{
    static bool temp_init_done = false;
    static temperature_sensor_handle_t temp_handle;

    if (!temp_init_done) {
        temperature_sensor_config_t temp_sensor_config = {
            // Typical default range for ambient
            .range_min = 0,
            .range_max = 50,
        };
        ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
        temp_init_done = true;
    }

    float temp_c = 25.0f;
    esp_err_t err = temperature_sensor_get_celsius(temp_handle, &temp_c);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Temp read failed, using 25C fallback");
        temp_c = 25.0f;
    }

    // Clamp to [0, 50] per lab instruction
    if (temp_c < 0.0f)  temp_c = 0.0f;
    if (temp_c > 50.0f) temp_c = 50.0f;

    return temp_c;
}

// --- Helper: measure distance (cm) using temperature-adjusted speed of sound ---
static float measure_distance_cm(float temp_c)
{
    double sum_cm = 0.0;
    int valid_samples = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        int64_t duration_us = ultrasonic_pulse_read_us();
        if (duration_us < 0) {
            // bad sample, skip
            continue;
        }

        // Convert duration to seconds (round trip)
        double t_sec = duration_us / 1e6;

        // Speed of sound vs temperature (m/s)
        // v = 331.3 + 0.606 * T(°C)
        double v_ms = 331.3 + 0.606 * (double)temp_c;

        // Distance (one way) in meters
        double d_m = (v_ms * t_sec) / 2.0;

        // Convert to cm
        double d_cm = d_m * 100.0;

        sum_cm += d_cm;
        valid_samples++;

        // Small delay between pulses to avoid ringing / echoes
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (valid_samples == 0) {
        return -1.0f;  // indicate failure
    }

    float avg_cm = (float)(sum_cm / valid_samples);

    // Apply calibration offset if needed (PCB to sensor face, etc.)
    avg_cm -= PCB_OFFSET_CM;

    return avg_cm;
}

void app_main(void)
{
    ultrasonic_gpio_init();
    ESP_LOGI(TAG, "Lab 6.1 Ultrasonic + Temperature starting...");

    ESP_LOGI(TAG, "Initial ECHO state: %d", gpio_get_level(ECHO_GPIO));
    ESP_LOGI(TAG, "Initial TRIG state: %d", gpio_get_level(TRIG_GPIO));
    
    // Test: manually pulse TRIG and watch ECHO
    for (int i = 0; i < 5; i++) {
        gpio_set_level(TRIG_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "Before pulse - ECHO: %d", gpio_get_level(ECHO_GPIO));
        
        gpio_set_level(TRIG_GPIO, 1);
        esp_rom_delay_us(10);
        gpio_set_level(TRIG_GPIO, 0);
        
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "After pulse - ECHO: %d", gpio_get_level(ECHO_GPIO));
    }

    while (1) {
        float temp_c = read_temperature_celsius();
        float distance_cm = measure_distance_cm(temp_c);

        if (distance_cm < 0.0f) {
            printf("Distance: ERROR (no echo) at %.1fC\n", temp_c);
        } else {
            // Lab format: "Distance: 3.50 cm at 23.0C"
            printf("Distance: %.2f cm at %.1fC\n", distance_cm, temp_c);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // once per second
    }
}