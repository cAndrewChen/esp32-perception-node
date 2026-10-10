#include "imu.h"

#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "peripherals.h"

static const char *TAG = "imu";

// MPU6050 defaults: gyro +/-250 dps -> 131 LSB per dps, accel +/-2 g -> 16384 LSB per g.
static constexpr float GYRO_LSB_PER_DPS = 131.0f;
static constexpr float ACCEL_LSB_PER_G = 16384.0f;
static constexpr float RAD_TO_DEG = 57.29578f;
static constexpr float MAX_DT_S = 0.2f;  // a longer gap is treated as a stall, not real elapsed motion

static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

struct RawSample {
    float ax, ay, az;  // g
    float gx, gy, gz;  // deg/s, bias not removed
};

static float s_bias[3];  // gyro bias x, y, z in deg/s
static ImuState s_state;
static int64_t s_last_us;
static bool s_initialized;

// Burst-read 14 bytes: accel (6), temperature (2, skipped), gyro (6). Values are big-endian int16.
static esp_err_t read_raw(RawSample &s)
{
    uint8_t b[14];
    esp_err_t err = imu_read_reg(REG_ACCEL_XOUT_H, b, sizeof(b));
    if (err != ESP_OK) return err;
    auto i16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
    s.ax = i16(0) / ACCEL_LSB_PER_G;
    s.ay = i16(2) / ACCEL_LSB_PER_G;
    s.az = i16(4) / ACCEL_LSB_PER_G;
    s.gx = i16(8) / GYRO_LSB_PER_DPS;
    s.gy = i16(10) / GYRO_LSB_PER_DPS;
    s.gz = i16(12) / GYRO_LSB_PER_DPS;
    return ESP_OK;
}

bool imu_calibrate()
{
    for (int attempt = 1; attempt <= IMU_CAL_MAX_ATTEMPTS; attempt++) {
        double sum[3] = {0, 0, 0}, sum_sq[3] = {0, 0, 0};
        int n = 0;
        for (int i = 0; i < IMU_CAL_SAMPLES; i++) {
            RawSample s;
            if (read_raw(s) == ESP_OK) {
                const float g[3] = {s.gx, s.gy, s.gz};
                for (int k = 0; k < 3; k++) {
                    sum[k] += g[k];
                    sum_sq[k] += (double)g[k] * g[k];
                }
                n++;
            }
            vTaskDelay(pdMS_TO_TICKS(IMU_CAL_SAMPLE_MS));
        }
        if (n < IMU_CAL_SAMPLES / 2) {
            ESP_LOGW(TAG, "calibration attempt %d: only %d good reads", attempt, n);
            continue;
        }

        float mean[3], stddev[3], worst = 0;
        for (int k = 0; k < 3; k++) {
            mean[k] = (float)(sum[k] / n);
            double var = sum_sq[k] / n - (double)mean[k] * mean[k];
            stddev[k] = var > 0 ? (float)sqrt(var) : 0.0f;
            if (stddev[k] > worst) worst = stddev[k];
        }
        if (worst > IMU_CAL_MAX_STDDEV_DPS) {
            ESP_LOGW(TAG, "calibration attempt %d rejected: stddev %.2f dps > %.2f (board moved?)",
                     attempt, worst, IMU_CAL_MAX_STDDEV_DPS);
            continue;
        }
        for (int k = 0; k < 3; k++) s_bias[k] = mean[k];
        ESP_LOGI(TAG, "gyro bias dps: x=%.3f y=%.3f z=%.3f (worst stddev %.3f)", mean[0], mean[1], mean[2], worst);
        return true;
    }
    for (float &b : s_bias) b = 0.0f;
    ESP_LOGW(TAG, "calibration failed after %d attempts, using zero bias", IMU_CAL_MAX_ATTEMPTS);
    return false;
}

esp_err_t imu_update(ImuState &out)
{
    RawSample s;
    esp_err_t err = read_raw(s);
    if (err != ESP_OK) return err;

    int64_t now = esp_timer_get_time();
    float dt = s_initialized ? (now - s_last_us) * 1e-6f : 0.0f;
    if (dt > MAX_DT_S) dt = MAX_DT_S;
    s_last_us = now;

    const float gx = s.gx - s_bias[0];
    const float gy = s.gy - s_bias[1];
    const float gz = s.gz - s_bias[2];

    // Tilt from gravity. Axis signs assume the chip's own X/Y/Z; remounting may flip them.
    const float accel_roll = atan2f(s.ay, s.az) * RAD_TO_DEG;
    const float accel_pitch = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD_TO_DEG;

    if (!s_initialized) {
        // Start from the accelerometer so the filter does not have to converge from zero.
        s_state.roll_deg = accel_roll;
        s_state.pitch_deg = accel_pitch;
        s_state.yaw_deg = 0.0f;
        s_initialized = true;
    } else {
        s_state.roll_deg = IMU_ALPHA * (s_state.roll_deg + gx * dt) + (1.0f - IMU_ALPHA) * accel_roll;
        s_state.pitch_deg = IMU_ALPHA * (s_state.pitch_deg + gy * dt) + (1.0f - IMU_ALPHA) * accel_pitch;
        s_state.yaw_deg += gz * dt;
    }
    s_state.yaw_rate_dps = gz;
    out = s_state;
    return ESP_OK;
}
