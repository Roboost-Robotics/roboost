// Wheel velocity controller: PI(D) on the speed error plus feedforward.
// Output is a signed duty cycle in [-1, 1].
#pragma once

struct PidGains {
    float kp;  // duty per rad/s of error
    float ki;  // duty per rad of accumulated error
    float kd;  // duty per rad/s^2 (on measurement, avoids setpoint kick)
    float kf;  // feedforward: duty per rad/s of setpoint (~1 / no-load speed at full duty)
    float ks;  // static friction / driver deadband: duty added in the setpoint direction
};

class Pid {
public:
    explicit Pid(const PidGains& gains) : g_(gains) {}

    float update(float setpoint, float measured, float dt) {
        const float error = setpoint - measured;
        const float derivative = has_prev_ ? -(measured - prev_measured_) / dt : 0.0f;
        prev_measured_ = measured;
        has_prev_ = true;

        const float static_ff = setpoint > 0.0f ? g_.ks : (setpoint < 0.0f ? -g_.ks : 0.0f);
        const float unclamped = g_.kf * setpoint + static_ff + g_.kp * error + g_.ki * integral_ + g_.kd * derivative;
        const float out = clamp(unclamped, -1.0f, 1.0f);

        // Anti-windup: stop integrating while saturated, unless the error pulls out of saturation.
        if (unclamped == out || (unclamped > out) != (error > 0.0f)) {
            integral_ += error * dt;
            if (g_.ki > 0.0f) {
                integral_ = clamp(integral_, -1.0f / g_.ki, 1.0f / g_.ki);
            }
        }
        return out;
    }

    void reset() {
        integral_ = 0.0f;
        has_prev_ = false;
    }

private:
    static float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

    PidGains g_;
    float integral_ = 0.0f;
    float prev_measured_ = 0.0f;
    bool has_prev_ = false;
};
