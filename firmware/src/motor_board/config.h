// Motor board: ESP32 DevKit V1 + 4x L298N channels (via level shifters) + 4x quadrature encoders.
#pragma once

#include <stdint.h>

#include "pid.h"

namespace config {

constexpr int STATUS_LED = 2;

// Geometry [m]. The wheels are 0.075 m physically; 0.06 is the value the robot was calibrated with.
constexpr float WHEEL_RADIUS = 0.06f;
constexpr float WHEEL_BASE = 0.3185f;   // front-rear axle distance
constexpr float TRACK_WIDTH = 0.38f;    // left-right wheel distance

struct Wheel {
    const char* joint;   // URDF joint name
    uint8_t in1, in2, ena, pwm_channel;
    uint8_t enc_a, enc_b;
    uint16_t encoder_ppr;  // encoder pulses per wheel revolution (one channel, one edge)
    bool invert;           // mirrored mounting: flips motor and encoder so + = forward
};

// Order: front-left, front-right, back-left, back-right (matches the kinematics).
// Wheel spins backwards on +cmd_vel.x -> flip `invert`.
// Wheel runs away at full speed     -> encoder counts the wrong way, swap enc_a/enc_b.
constexpr Wheel WHEELS[4] = {
    {"wheel_front_left_joint", 23, 22, 21, 0, 17, 16, 360, false},
    {"wheel_front_right_joint", 14, 18, 19, 1, 5, 15, 600, true},
    {"wheel_back_left_joint", 26, 27, 13, 2, 39, 36, 360, false},
    {"wheel_back_right_joint", 32, 33, 25, 3, 35, 34, 360, true},
};

constexpr uint32_t PWM_FREQUENCY = 15000;  // Hz
constexpr uint8_t PWM_BITS = 10;

// Velocity loop (rad/s at the wheel -> duty). Defaults settle in <0.5 s on a simulated
// plant over a wide range of motor gains/deadbands. Tuning on the stand: ks just below the
// duty at which the wheel starts turning, kf so it roughly reaches the setpoint with
// kp=ki=0, then kp/ki for disturbance rejection (watch joint_states velocity vs. effort).
constexpr PidGains PID = {0.1f /*kp*/, 1.0f /*ki*/, 0.0f /*kd*/, 0.04f /*kf*/, 0.2f /*ks*/};

constexpr uint32_t CONTROL_PERIOD_MS = 20;   // 50 Hz
constexpr float VELOCITY_FILTER = 0.5f;      // low pass on measured speed (1 = no filtering)
constexpr float MAX_WHEEL_SPEED = 15.0f;     // rad/s, commands are scaled down to this
constexpr float MAX_WHEEL_ACCEL = 30.0f;     // rad/s^2, setpoint ramp
constexpr uint32_t CMD_VEL_TIMEOUT_MS = 500; // stop if cmd_vel goes quiet

constexpr uint32_t PUBLISH_PERIOD_MS = 50;   // joint_states + wheel_odom at 20 Hz

}  // namespace config
