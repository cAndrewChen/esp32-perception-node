#pragma once

#include <stdint.h>
#include "esp_err.h"

// HC-SR04 ultrasonic sensor. Keep pins in sync with diagram.json.
constexpr int PIN_SONAR_TRIG = 5;
constexpr int PIN_SONAR_ECHO = 6;

constexpr int   SONAR_PERIOD_MS = 60;     // trigger-to-trigger, long enough for old echoes to die out
constexpr int   SONAR_TIMEOUT_MS = 30;    // 400 cm is about 23 ms; longer means no echo
constexpr float SONAR_MIN_CM = 2.0f;      // outside 2..400 cm is treated as noise
constexpr float SONAR_MAX_CM = 400.0f;
constexpr float SONAR_US_PER_CM = 58.0f;  // round trip

struct DistanceReading {
    bool has_value;     // false until the first valid echo; nothing is made up before that
    bool fresh;         // true if the last attempt produced a new valid reading, false if cm is held
    float cm;           // last valid distance
    uint32_t misses;    // consecutive attempts without a valid echo
    int64_t t_us;       // time of the last valid reading
};

// Configures the pins and starts the measuring task.
esp_err_t distance_start();

// Latest reading (copy). When no echo comes back, cm keeps its last valid value and fresh is false.
DistanceReading distance_get();
