#pragma once

#include "esp_err.h"

// Complementary filter output. Angles in degrees, rate in degrees per second.
struct ImuState {
    float pitch_deg;
    float roll_deg;
    float yaw_deg;       // gyro integration only, drifts over time
    float yaw_rate_dps;  // bias-corrected, used by the servo feedforward
};

// Blend ratio: how much of each step comes from the gyro-integrated angle.
constexpr float IMU_ALPHA = 0.98f;

// Gyro bias calibration at startup. The board must be still.
constexpr int   IMU_CAL_SAMPLES = 200;
constexpr int   IMU_CAL_SAMPLE_MS = 5;
constexpr float IMU_CAL_MAX_STDDEV_DPS = 1.0f;  // above this the board was probably moved
constexpr int   IMU_CAL_MAX_ATTEMPTS = 3;

// Returns true if a calibration was accepted. After all attempts fail it falls back to
// zero bias, logs a warning and returns false.
bool imu_calibrate();

// Reads the sensor and advances the filter. dt is measured from the previous call.
esp_err_t imu_update(ImuState &out);
