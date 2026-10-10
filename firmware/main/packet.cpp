#include "packet.h"

#include <math.h>

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static int16_t to_i16(float value, float scale)
{
    float v = roundf(value * scale);
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
}

static float wrap_180(float deg)
{
    return deg - 360.0f * floorf((deg + 180.0f) / 360.0f);
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

void packet_pack(const TelemetrySample &s, uint8_t *out)
{
    out[0] = PACKET_MAGIC0;
    out[1] = PACKET_MAGIC1;
    out[2] = (s.obj_found ? PACKET_FLAG_OBJ_FOUND : 0) | (s.dist_valid ? PACKET_FLAG_DIST_VALID : 0) |
             (s.dist_fresh ? PACKET_FLAG_DIST_FRESH : 0);
    out[3] = (uint8_t)(s.t_ms & 0xFF);
    out[4] = (uint8_t)((s.t_ms >> 8) & 0xFF);
    out[5] = (uint8_t)((s.t_ms >> 16) & 0xFF);
    out[6] = (uint8_t)((s.t_ms >> 24) & 0xFF);

    float dist = s.dist_valid ? s.dist_cm * 10.0f : 0.0f;
    if (dist < 0.0f) dist = 0.0f;
    if (dist > 65535.0f) dist = 65535.0f;
    put16(out + 7, (uint16_t)roundf(dist));

    put16(out + 9, (uint16_t)to_i16(s.pitch_deg, 100.0f));
    put16(out + 11, (uint16_t)to_i16(s.roll_deg, 100.0f));
    put16(out + 13, (uint16_t)to_i16(wrap_180(s.yaw_deg), 100.0f));
    put16(out + 15, (uint16_t)to_i16(s.servo_deg, 100.0f));
    put16(out + 17, (uint16_t)to_i16(s.obj_found ? s.obj_x_px : 0.0f, 100.0f));
    put16(out + 19, (uint16_t)to_i16(s.obj_found ? s.obj_y_px : 0.0f, 100.0f));

    put16(out + 21, crc16_ccitt(out + 2, 19));
}
