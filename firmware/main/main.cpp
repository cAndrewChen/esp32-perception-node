#include <atomic>
#include <math.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "imu.h"
#include "peripherals.h"
#include "pid.h"
#include "vision.h"
#include "world.h"

static const char *TAG = "main";

// Camera runs at about 30 fps, control at 50 Hz (CONTROL_PERIOD_MS).
constexpr int CAMERA_PERIOD_MS = 33;

// PID: error is dx in pixels, output is the servo angle in degrees.
constexpr float PID_KP = 0.1f;
constexpr float PID_KI = 0.1f;
constexpr float PID_KD = 0.0f;

// Success: object within TRACK_TOL_PX of the crosshair for TRACK_HOLD_S seconds.
constexpr float TRACK_TOL_PX = 20.0f;
constexpr float TRACK_HOLD_S = 0.5f;

static QueueHandle_t s_frame_q;          // length 1, holds the newest FrameResult
static std::atomic<float> s_servo_deg{0.0f};
static ImuState s_imu_shared;
static portMUX_TYPE s_imu_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_frame[FRAME_W * FRAME_H];

static ImuState read_imu()
{
    ImuState s;
    portENTER_CRITICAL(&s_imu_mux);
    s = s_imu_shared;
    portEXIT_CRITICAL(&s_imu_mux);
    return s;
}

// IMU task: reads and filters the IMU every control period and publishes the latest state.
static void imu_task(void *)
{
    TickType_t last = xTaskGetTickCount();
    while (true) {
        ImuState s;
        if (imu_update(s) == ESP_OK) {
            portENTER_CRITICAL(&s_imu_mux);
            s_imu_shared = s;
            portEXIT_CRITICAL(&s_imu_mux);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

// Perception task: renders the simulated scene, finds the object, and overwrites the queue with the result.
// It also measures the true tracking error for the pass check.
static void perception_task(void *)
{
    world_init();
    const int64_t t0 = esp_timer_get_time();
    TickType_t last = xTaskGetTickCount();
    int64_t in_tol_since_us = -1;
    int frames = 0, frames_in_tol = 0;

    while (true) {
        const int64_t now = esp_timer_get_time();
        const float cam_deg = s_servo_deg.load() + read_imu().yaw_deg;  // camera points where servo + base point
        const ObjectPos obj = world_object((now - t0) * 1e-6f);

        world_render(s_frame, cam_deg, obj);
        const int64_t render_done = esp_timer_get_time();
        FrameResult fr = vision_find(s_frame);
        fr.t_us = now;
        xQueueOverwrite(s_frame_q, &fr);
        const int64_t render_us = render_done - now;
        const int64_t frame_us = esp_timer_get_time() - now;

        const float true_err_px = PX_PER_DEG * (obj.x_deg - cam_deg);
        const bool in_tol = fabsf(true_err_px) <= TRACK_TOL_PX;
        if (in_tol) {
            if (in_tol_since_us < 0) in_tol_since_us = now;
            frames_in_tol++;
        } else {
            in_tol_since_us = -1;
        }
        frames++;

        if (frames % 30 == 0) {
            const float held_s = in_tol_since_us < 0 ? 0.0f : (now - in_tol_since_us) * 1e-6f;
            ESP_LOGI(TAG, "track: wanted=%.1f actual=%.1f deg, err=%.1f px, in-tol %.2f s, %s, %d%% of frames in tol, found=%d, frame %lld us (render %lld)",
                     obj.x_deg, cam_deg, true_err_px, held_s, held_s >= TRACK_HOLD_S ? "LOCKED" : "not locked",
                     100 * frames_in_tol / frames, fr.found, (long long)frame_us, (long long)render_us);
        }
        // If a frame took longer than the period, vTaskDelayUntil returns without blocking. Always
        // sleep at least one tick so the idle task can run and feed the watchdog.
        if (xTaskDelayUntil(&last, pdMS_TO_TICKS(CAMERA_PERIOD_MS)) == pdFALSE) {
            vTaskDelay(1);
            last = xTaskGetTickCount();  // resync instead of falling further behind
        }
    }
}

// Control task: every 50 Hz tick, accumulate the yaw-rate feedforward; when a new frame is waiting in the
// queue, run the PID once on it. The command is pid_out + ff, clamped to the servo range.
static void control_task(void *)
{
    Pid pid = {PID_KP, PID_KI, PID_KD, -SERVO_MAX_DEG, SERVO_MAX_DEG};
    float pid_out = 0.0f;
    float ff = 0.0f;  // persistent: it is the base's accumulated rotation, never reset
    float last_dx = 0.0f;
    int64_t last_frame_us = -1;
    int64_t last_tick_us = esp_timer_get_time();
    int tick = 0;
    TickType_t last = xTaskGetTickCount();

    while (true) {
        const int64_t now = esp_timer_get_time();
        const float dt = (now - last_tick_us) * 1e-6f;
        last_tick_us = now;

        // Base turning one way means the camera must turn the other way.
        ff -= read_imu().yaw_rate_dps * dt;

        FrameResult fr;
        if (xQueueReceive(s_frame_q, &fr, 0) == pdTRUE && fr.found) {
            const float dt_frame = last_frame_us < 0 ? 0.0f : (fr.t_us - last_frame_us) * 1e-6f;
            last_frame_us = fr.t_us;
            last_dx = fr.cx - FRAME_CX;
            pid_out = pid.update(last_dx, dt_frame);
        }

        float cmd = pid_out + ff;
        if (cmd > SERVO_MAX_DEG) cmd = SERVO_MAX_DEG;
        if (cmd < -SERVO_MAX_DEG) cmd = -SERVO_MAX_DEG;
        servo_set_angle(cmd);
        s_servo_deg.store(cmd);

        if (++tick % 50 == 0) {
            ESP_LOGI(TAG, "ctl: dx=%.1f pid=%.1f ff=%.1f cmd=%.1f", last_dx, pid_out, ff, cmd);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(telemetry_init());
    ESP_ERROR_CHECK(servo_init());
    imu_calibrate();

    s_frame_q = xQueueCreate(1, sizeof(FrameResult));
    ESP_LOGI(TAG, "starting tasks: camera %d ms, control %d Hz", CAMERA_PERIOD_MS, CONTROL_HZ);
    xTaskCreate(imu_task, "imu", 4096, nullptr, 5, nullptr);
    xTaskCreate(perception_task, "perception", 4096, nullptr, 4, nullptr);
    xTaskCreate(control_task, "control", 4096, nullptr, 6, nullptr);
}
