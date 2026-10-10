#pragma once

#include <stdint.h>

// HSV threshold: hue in degrees (0..359, circular), saturation and value in 0..255.
constexpr int VISION_HUE_CENTER_DEG = 0;     // red
constexpr int VISION_HUE_HALF_WIDTH_DEG = 15;
constexpr int VISION_SAT_MIN = 128;
constexpr int VISION_VAL_MIN = 77;
constexpr int VISION_MIN_PIXELS = 10;        // fewer matches than this counts as "no object"

struct FrameResult {
    bool found;
    float cx, cy;   // centroid in pixels, valid when found
    int count;      // number of pixels that passed the threshold
    int64_t t_us;   // capture time, set by the caller
};

// Thresholds an RGB565 frame of FRAME_W x FRAME_H and returns the centroid of the matching pixels.
FrameResult vision_find(const uint16_t *frame);
