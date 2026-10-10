#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu.h"
#include "peripherals.h"

static const char *TAG = "main";

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(telemetry_init());
    ESP_ERROR_CHECK(servo_init());
    ESP_LOGI(TAG, "peripherals ready, control loop %d Hz", CONTROL_HZ);

    imu_calibrate();

    // Bring-up check only: sweep the servo, send a test string, log the filter. Replaced by the real tasks later.
    const char hello[] = "telemetry up\n";
    float angle = -45.0f;
    float step = 5.0f;
    int tick = 0;
    while (true) {
        servo_set_angle(angle);
        telemetry_write((const uint8_t *)hello, sizeof(hello) - 1);

        ImuState imu;
        if (imu_update(imu) == ESP_OK && tick++ % 25 == 0) {
            ESP_LOGI(TAG, "pitch=%.1f roll=%.1f yaw=%.1f yaw_rate=%.2f",
                     imu.pitch_deg, imu.roll_deg, imu.yaw_deg, imu.yaw_rate_dps);
        }

        angle += step;
        if (angle > 45.0f || angle < -45.0f) step = -step;
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
