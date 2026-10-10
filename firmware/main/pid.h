#pragma once

// PID with measured dt, output clamp and conditional-integration anti-windup.
// Units are whatever the caller uses (here: error in pixels, output in degrees).
struct Pid {
    float kp, ki, kd;
    float out_min, out_max;
    float integral_ = 0.0f;
    float prev_err_ = 0.0f;
    bool has_prev_ = false;

    float update(float err, float dt)
    {
        float d = 0.0f;
        if (has_prev_ && dt > 0.0f) d = (err - prev_err_) / dt;
        prev_err_ = err;
        has_prev_ = true;

        float candidate = integral_ + (dt > 0.0f ? err * dt : 0.0f);
        float unsat = kp * err + ki * candidate + kd * d;
        float out = unsat > out_max ? out_max : (unsat < out_min ? out_min : unsat);

        // Only keep integrating if the output is not saturated, or the error pulls it back out.
        bool saturated_high = unsat > out_max && err > 0.0f;
        bool saturated_low = unsat < out_min && err < 0.0f;
        if (!saturated_high && !saturated_low) integral_ = candidate;
        return out;
    }
};
