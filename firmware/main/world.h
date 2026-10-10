#pragma once

#include <stdint.h>

// Simulated camera. Wokwi has no camera, so we draw the frame ourselves.
constexpr int   FRAME_W = 160;
constexpr int   FRAME_H = 120;
constexpr float PX_PER_DEG = 1.0f;
constexpr float FRAME_CX = (FRAME_W - 1) / 2.0f;  // the crosshair
constexpr float FRAME_CY = (FRAME_H - 1) / 2.0f;
constexpr float OBJ_RADIUS_PX = 8.0f;

// The object follows a spiral in (x, y) degrees: the radius oscillates between
// MID - AMP and MID + AMP while the angle sweeps around. Phases and speeds are random per boot.
// x is what the pan servo can follow; y only moves the object up and down in the frame.
constexpr float SPIRAL_R_MID_DEG = 40.0f;
constexpr float SPIRAL_R_AMP_DEG = 15.0f;
constexpr float SPIRAL_Y_SCALE = 0.5f;

struct ObjectPos {
    float x_deg;
    float y_deg;
};

void world_init();                       // picks the random spiral
ObjectPos world_object(float t_s);       // object position at time t (seconds since start)

// Draws the object as seen by a camera pointing at cam_deg into an RGB565 buffer of FRAME_W x FRAME_H.
void world_render(uint16_t *buf, float cam_deg, const ObjectPos &obj);
