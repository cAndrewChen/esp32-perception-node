#include "vision.h"

#include <stdlib.h>
#include "world.h"

// Integer RGB -> HSV test, so a 160x120 frame stays cheap at 30 fps.
static bool matches(uint16_t p)
{
    int r = (p >> 11) & 0x1F, g = (p >> 5) & 0x3F, b = p & 0x1F;
    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);

    int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
    int delta = mx - mn;
    if (mx < VISION_VAL_MIN || delta == 0) return false;
    if (delta * 255 / mx < VISION_SAT_MIN) return false;

    int h;
    if (mx == r)      h = 60 * (g - b) / delta;
    else if (mx == g) h = 120 + 60 * (b - r) / delta;
    else              h = 240 + 60 * (r - g) / delta;
    if (h < 0) h += 360;

    int d = abs(h - VISION_HUE_CENTER_DEG);
    if (d > 180) d = 360 - d;
    return d <= VISION_HUE_HALF_WIDTH_DEG;
}

FrameResult vision_find(const uint16_t *frame)
{
    long sum_x = 0, sum_y = 0;
    int count = 0;
    for (int y = 0; y < FRAME_H; y++) {
        for (int x = 0; x < FRAME_W; x++) {
            if (matches(frame[y * FRAME_W + x])) {
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
    }
    FrameResult r = {};
    r.count = count;
    if (count >= VISION_MIN_PIXELS) {
        r.found = true;
        r.cx = (float)sum_x / count;
        r.cy = (float)sum_y / count;
    }
    return r;
}
