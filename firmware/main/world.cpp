#include "world.h"

#include <math.h>
#include "esp_random.h"

static constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static constexpr uint16_t BACKGROUND = rgb565(20, 20, 20);
static constexpr uint16_t OBJECT_COLOR = rgb565(230, 40, 40);  // red

static float s_theta0, s_omega_theta, s_phase_r, s_omega_r;

static float rand_range(float lo, float hi)
{
    return lo + (hi - lo) * (esp_random() / 4294967295.0f);
}

void world_init()
{
    s_theta0 = rand_range(0.0f, 2.0f * (float)M_PI);
    // s_omega_theta = rand_range(0.5f, 0.75f);  // rad/s
    s_omega_theta = 0.7f;
    s_phase_r = rand_range(0.0f, 2.0f * (float)M_PI);
    s_omega_r = rand_range(0.10f, 0.20f);
}

ObjectPos world_object(float t_s)
{
    float r = SPIRAL_R_MID_DEG + SPIRAL_R_AMP_DEG * sinf(s_omega_r * t_s + s_phase_r);
    float theta = s_theta0 + s_omega_theta * t_s;
    return {r * cosf(theta), r * sinf(theta) * SPIRAL_Y_SCALE};
}

// Kept out of line on purpose. Before this, about a third of frames were drawn as if the camera pointed at 0 deg
// even though a nonzero angle was passed in. The cause is not proven: this loop takes a few ms, so other tasks
// often preempt it, and a float held in a register across it is the likely thing that was lost. Doing the clear
// in its own call forces cam_deg onto the stack. main.cpp checks for the symptom and logs a warning.
static void __attribute__((noinline)) clear_frame(uint16_t *buf)
{
    for (int i = 0; i < FRAME_W * FRAME_H; i++) buf[i] = BACKGROUND;
}

void world_render(uint16_t *buf, float cam_deg, const ObjectPos &obj)
{
    clear_frame(buf);

    const float px = FRAME_CX + PX_PER_DEG * (obj.x_deg - cam_deg);
    const float py = FRAME_CY + PX_PER_DEG * obj.y_deg;
    int x0 = (int)floorf(px - OBJ_RADIUS_PX), x1 = (int)ceilf(px + OBJ_RADIUS_PX);
    int y0 = (int)floorf(py - OBJ_RADIUS_PX), y1 = (int)ceilf(py + OBJ_RADIUS_PX);
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > FRAME_W - 1) x1 = FRAME_W - 1;
    if (y1 > FRAME_H - 1) y1 = FRAME_H - 1;

    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            float dx = x - px, dy = y - py;
            if (dx * dx + dy * dy <= OBJ_RADIUS_PX * OBJ_RADIUS_PX) buf[y * FRAME_W + x] = OBJECT_COLOR;
        }
    }
}
