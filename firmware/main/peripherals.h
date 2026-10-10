#pragma once

#include <stdint.h>
#include "esp_err.h"

// Pin map. Keep in sync with diagram.json (Wokwi wiring).
constexpr int PIN_I2C_SDA = 8;
constexpr int PIN_I2C_SCL = 9;
constexpr int PIN_UART_TX = 17;
constexpr int PIN_UART_RX = 18;
constexpr int PIN_SERVO   = 4;

// Control loop rate: 50 Hz (20 ms), matching the servo's 20 ms PWM period.
constexpr int CONTROL_HZ = 50;
constexpr int CONTROL_PERIOD_MS = 1000 / CONTROL_HZ;

// I2C / MPU6050
constexpr uint8_t MPU6050_ADDR = 0x68;
esp_err_t i2c_init();
esp_err_t imu_init();                          // wakes the MPU6050
esp_err_t imu_read_reg(uint8_t reg, uint8_t *buf, size_t len);

// UART telemetry (UART1, not the UART0 console)
constexpr int TELEMETRY_BAUD = 115200;
esp_err_t telemetry_init();
int telemetry_write(const uint8_t *data, size_t len);

// Servo, angle in degrees relative to center, clamped to +/- SERVO_MAX_DEG
constexpr float SERVO_MAX_DEG = 90.0f;
esp_err_t servo_init();
esp_err_t servo_set_angle(float deg);
