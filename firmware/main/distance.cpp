#include "distance.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "distance";

static TaskHandle_t s_task;
static volatile int64_t s_rise_us;
static volatile int64_t s_fall_us;

static DistanceReading s_reading;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// Runs on every ECHO edge. Rising edge: the sound has just left. Falling edge: the echo came back.
static void IRAM_ATTR echo_isr(void *)
{
    const int64_t now = esp_timer_get_time();
    if (gpio_get_level((gpio_num_t)PIN_SONAR_ECHO)) {
        s_rise_us = now;
    } else {
        s_fall_us = now;
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_task, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

DistanceReading distance_get()
{
    portENTER_CRITICAL(&s_mux);
    DistanceReading r = s_reading;
    portEXIT_CRITICAL(&s_mux);
    return r;
}

static void publish(bool ok, float cm, int64_t t_us)
{
    portENTER_CRITICAL(&s_mux);
    if (ok) {
        s_reading.has_value = true;
        s_reading.fresh = true;
        s_reading.cm = cm;
        s_reading.misses = 0;
        s_reading.t_us = t_us;
    } else {
        s_reading.fresh = false;  // cm and t_us stay as they were
        s_reading.misses++;
    }
    portEXIT_CRITICAL(&s_mux);
}

static void distance_task(void *)
{
    // The ISR notifies this task, so the handle must be set before the handler is attached.
    s_task = xTaskGetCurrentTaskHandle();
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)PIN_SONAR_ECHO, echo_isr, nullptr));

    TickType_t last = xTaskGetTickCount();
    int cycle = 0;
    while (true) {
        s_rise_us = 0;
        s_fall_us = 0;
        ulTaskNotifyTake(pdTRUE, 0);  // drop a notification left over from a late echo

        gpio_set_level((gpio_num_t)PIN_SONAR_TRIG, 1);
        esp_rom_delay_us(10);
        gpio_set_level((gpio_num_t)PIN_SONAR_TRIG, 0);

        bool ok = false;
        float cm = 0.0f;
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SONAR_TIMEOUT_MS)) > 0 && s_rise_us > 0 && s_fall_us > s_rise_us) {
            cm = (s_fall_us - s_rise_us) / SONAR_US_PER_CM;
            ok = cm >= SONAR_MIN_CM && cm <= SONAR_MAX_CM;
        }
        publish(ok, cm, s_fall_us);

        if (++cycle % 16 == 0) {  // about once per second
            DistanceReading r = distance_get();
            if (!r.has_value) {
                ESP_LOGI(TAG, "no echo yet");
            } else {
                ESP_LOGI(TAG, "dist: %.1f cm (%s, %u misses in a row)", r.cm, r.fresh ? "fresh" : "held",
                         (unsigned)r.misses);
            }
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SONAR_PERIOD_MS));
    }
}

esp_err_t distance_start()
{
    gpio_config_t trig = {};
    trig.pin_bit_mask = 1ULL << PIN_SONAR_TRIG;
    trig.mode = GPIO_MODE_OUTPUT;
    ESP_RETURN_ON_ERROR(gpio_config(&trig), TAG, "trig config");
    gpio_set_level((gpio_num_t)PIN_SONAR_TRIG, 0);

    gpio_config_t echo = {};
    echo.pin_bit_mask = 1ULL << PIN_SONAR_ECHO;
    echo.mode = GPIO_MODE_INPUT;
    echo.pull_down_en = GPIO_PULLDOWN_ENABLE;
    echo.intr_type = GPIO_INTR_ANYEDGE;
    ESP_RETURN_ON_ERROR(gpio_config(&echo), TAG, "echo config");

    // gpio_install_isr_service fails with ESP_ERR_INVALID_STATE if already installed; that is fine.
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    return xTaskCreate(distance_task, "distance", 3072, nullptr, 3, nullptr) == pdPASS ? ESP_OK : ESP_FAIL;
}
