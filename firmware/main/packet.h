#pragma once

#include <stddef.h>
#include <stdint.h>

// Telemetry packet, all multi-byte fields little-endian, 23 bytes total:
//
//   offset size field
//   0      2    magic          0xAA 0x55
//   2      1    flags          bit0 object found, bit1 distance has a value, bit2 distance is fresh (not held)
//   3      4    t_ms           uint32, milliseconds since boot
//   7      2    distance       uint16, 0.1 cm units (0 when bit1 is clear)
//   9      2    pitch          int16, 0.01 deg
//   11     2    roll           int16, 0.01 deg
//   13     2    yaw            int16, 0.01 deg, wrapped to +/-180 (gyro-only, drifts)
//   15     2    servo_angle    int16, 0.01 deg
//   17     2    obj_x          int16, 0.01 px, offset from the crosshair, right is positive (0 if not found)
//   19     2    obj_y          int16, 0.01 px, offset from the crosshair, down is positive (0 if not found)
//   21     2    crc16          CRC-16/CCITT-FALSE over bytes 2..20
constexpr uint8_t PACKET_MAGIC0 = 0xAA;
constexpr uint8_t PACKET_MAGIC1 = 0x55;
constexpr size_t  PACKET_LEN = 23;

constexpr uint8_t PACKET_FLAG_OBJ_FOUND = 1 << 0;
constexpr uint8_t PACKET_FLAG_DIST_VALID = 1 << 1;
constexpr uint8_t PACKET_FLAG_DIST_FRESH = 1 << 2;

struct TelemetrySample {
    uint32_t t_ms;
    bool obj_found;
    float obj_x_px, obj_y_px;
    bool dist_valid, dist_fresh;
    float dist_cm;
    float pitch_deg, roll_deg, yaw_deg;
    float servo_deg;
};

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection. crc16("123456789") == 0x29B1.
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

// Writes PACKET_LEN bytes into out. Out-of-range values saturate instead of wrapping.
void packet_pack(const TelemetrySample &s, uint8_t *out);
