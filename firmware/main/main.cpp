#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "peripherals.h"

static const char *TAG = "main";

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(telemetry_init());
    ESP_ERROR_CHECK(servo_init());
    ESP_LOGI(TAG, "peripherals ready, control loop %d Hz", CONTROL_HZ);

    // Bring-up check only: sweep the servo and send a test string. Replaced by the real tasks later.
    const char hello[] = "telemetry up\n";
    float angle = -45.0f;
    float step = 5.0f;
    while (true) {
        servo_set_angle(angle);
        telemetry_write((const uint8_t *)hello, sizeof(hello) - 1);
        angle += step;
        if (angle > 45.0f || angle < -45.0f) step = -step;
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
