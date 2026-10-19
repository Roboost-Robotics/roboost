// Mecanum kinematics. Wheel order: front-left, front-right, back-left, back-right.
// Wheel speeds in rad/s, positive = wheel rolls the robot forward (motor/encoder
// direction is handled per wheel in the board config).
#pragma once

struct Twist2D {
    float vx;  // m/s, forward
    float vy;  // m/s, left
    float wz;  // rad/s, counter-clockwise
};

class Mecanum {
public:
    // wheel_base: front-back axle distance, track_width: left-right wheel distance [m]
    Mecanum(float wheel_radius, float wheel_base, float track_width)
        : r_(wheel_radius), k_((wheel_base + track_width) / 2.0f) {}

    // Robot twist -> wheel speeds.
    void inverse(const Twist2D& t, float w[4]) const {
        w[0] = (t.vx - t.vy - k_ * t.wz) / r_;
        w[1] = (t.vx + t.vy + k_ * t.wz) / r_;
        w[2] = (t.vx + t.vy - k_ * t.wz) / r_;
        w[3] = (t.vx - t.vy + k_ * t.wz) / r_;
    }

    // Wheel speeds -> robot twist (least squares solution of the inverse).
    Twist2D forward(const float w[4]) const {
        return {
            r_ / 4.0f * (w[0] + w[1] + w[2] + w[3]),
            r_ / 4.0f * (-w[0] + w[1] + w[2] - w[3]),
            r_ / (4.0f * k_) * (-w[0] + w[1] - w[2] + w[3]),
        };
    }

private:
    float r_;
    float k_;
};
