#include "peripherals.h"

#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "esp_check.h"

static const char *TAG = "periph";

// ---------- I2C + IMU ----------

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_imu;

esp_err_t i2c_init()
{
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = (gpio_num_t)PIN_I2C_SDA;
    bus_cfg.scl_io_num = (gpio_num_t)PIN_I2C_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus), TAG, "i2c bus");

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = MPU6050_ADDR;
    dev_cfg.scl_speed_hz = 400000;
    return i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_imu);
}

esp_err_t imu_read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_imu, &reg, 1, buf, len, 100);
}

esp_err_t imu_init()
{
    // PWR_MGMT_1 (0x6B) = 0 clears the sleep bit; the chip powers up asleep.
    const uint8_t wake[] = {0x6B, 0x00};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(s_imu, wake, sizeof(wake), 100), TAG, "imu wake");

    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(imu_read_reg(0x75, &who, 1), TAG, "imu who_am_i");
    ESP_LOGI(TAG, "MPU6050 WHO_AM_I = 0x%02x", who);
    return who == MPU6050_ADDR ? ESP_OK : ESP_ERR_NOT_FOUND;
}

// ---------- UART telemetry ----------

static constexpr uart_port_t TELEMETRY_UART = TELEMETRY_ON_CONSOLE ? UART_NUM_0 : UART_NUM_1;

esp_err_t telemetry_init()
{
    if (TELEMETRY_ON_CONSOLE) {
        // The console already configured UART0 (115200 8N1). Only add a driver so we can write raw bytes;
        // printf would translate newlines and corrupt binary data.
        return uart_driver_install(TELEMETRY_UART, 256, 512, 0, nullptr, 0);
    }
    uart_config_t cfg = {};
    cfg.baud_rate = TELEMETRY_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    ESP_RETURN_ON_ERROR(uart_driver_install(TELEMETRY_UART, 256, 512, 0, nullptr, 0), TAG, "uart install");
    ESP_RETURN_ON_ERROR(uart_param_config(TELEMETRY_UART, &cfg), TAG, "uart config");
    return uart_set_pin(TELEMETRY_UART, PIN_UART_TX, PIN_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int telemetry_write(const uint8_t *data, size_t len)
{
    return uart_write_bytes(TELEMETRY_UART, data, len);
}

// ---------- LEDC servo ----------

// 1 tick = 1 us, 20 ms period (50 Hz). Pulse 500..2500 us spans 0..180 deg.
static constexpr uint32_t SERVO_TIMER_HZ = 1000000;
static constexpr uint32_t SERVO_PERIOD_TICKS = 20000;
static constexpr float SERVO_MIN_US = 500.0f;
static constexpr float SERVO_MAX_US = 2500.0f;

// Clamp the angle and convert it to a pulse width in microseconds.
static float servo_pulse_us(float deg)
{
    if (deg > SERVO_MAX_DEG) deg = SERVO_MAX_DEG;
    if (deg < -SERVO_MAX_DEG) deg = -SERVO_MAX_DEG;
    return SERVO_MIN_US + (deg + SERVO_MAX_DEG) / (2.0f * SERVO_MAX_DEG) * (SERVO_MAX_US - SERVO_MIN_US);
}

static constexpr uint32_t LEDC_DUTY_RES_BITS = 14;  // 16384 steps over the 20 ms period

esp_err_t servo_init()
{
    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_cfg.duty_resolution = (ledc_timer_bit_t)LEDC_DUTY_RES_BITS;
    timer_cfg.timer_num = LEDC_TIMER_0;
    timer_cfg.freq_hz = 50;
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "ledc timer");

    ledc_channel_config_t ch_cfg = {};
    ch_cfg.gpio_num = PIN_SERVO;
    ch_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_cfg.channel = LEDC_CHANNEL_0;
    ch_cfg.timer_sel = LEDC_TIMER_0;
    ch_cfg.duty = 0;
    ch_cfg.hpoint = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "ledc channel");
    return servo_set_angle(0.0f);
}

esp_err_t servo_set_angle(float deg)
{
    uint32_t duty = (uint32_t)(servo_pulse_us(deg) / 20000.0f * (1u << LEDC_DUTY_RES_BITS));
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty), TAG, "ledc duty");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}
