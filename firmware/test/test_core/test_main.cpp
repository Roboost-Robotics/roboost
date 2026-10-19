// Host-side tests for the hardware-free parts: pio test -e native
#include <unity.h>

#include <cmath>
#include <vector>

#include "mecanum.h"
#include "pid.h"
#include "rplidar.h"

void setUp() {}
void tearDown() {}

static const Mecanum kinematics(0.06f, 0.3185f, 0.38f);

static void test_forward_drive_turns_all_wheels_forward() {
    float w[4];
    kinematics.inverse({0.3f, 0.0f, 0.0f}, w);
    for (float v : w) {
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.0f, v);
    }
}

static void test_ccw_rotation_spins_right_side_forward() {
    float w[4];
    kinematics.inverse({0.0f, 0.0f, 1.0f}, w);
    TEST_ASSERT_TRUE(w[0] < 0 && w[2] < 0);  // left wheels backwards
    TEST_ASSERT_TRUE(w[1] > 0 && w[3] > 0);  // right wheels forwards
}

static void test_strafe_left_pattern() {
    float w[4];
    kinematics.inverse({0.0f, 0.2f, 0.0f}, w);
    TEST_ASSERT_TRUE(w[0] < 0 && w[1] > 0 && w[2] > 0 && w[3] < 0);
}

static void test_forward_inverts_inverse() {
    const Twist2D in = {0.21f, -0.13f, 0.7f};
    float w[4];
    kinematics.inverse(in, w);
    const Twist2D out = kinematics.forward(w);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, in.vx, out.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, in.vy, out.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, in.wz, out.wz);
}

// First order motor with deadband: the PID has to reach and hold the setpoint.
static void test_pid_tracks_setpoint_on_simulated_motor() {
    Pid pid({0.1f, 1.0f, 0.0f, 0.04f, 0.2f});
    const float dt = 0.02f, tau = 0.08f, gain = 22.0f, deadband = 0.2f;
    float speed = 0.0f, duty = 0.0f;
    for (int i = 0; i < 150; i++) {
        duty = pid.update(8.0f, speed, dt);
        const float drive = std::fabs(duty) > deadband ? gain * (duty - std::copysign(deadband, duty)) : 0.0f;
        speed += (drive - speed) * dt / tau;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 8.0f, speed);
    TEST_ASSERT_TRUE(duty > 0.0f && duty < 1.0f);
}

static void test_pid_output_is_clamped_and_does_not_wind_up() {
    Pid pid({0.1f, 1.0f, 0.0f, 0.04f, 0.2f});
    for (int i = 0; i < 500; i++) {
        TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.update(100.0f, 0.0f, 0.02f));  // stalled wheel
    }
    // As soon as the wheel overshoots, the output must come down immediately.
    TEST_ASSERT_TRUE(pid.update(5.0f, 6.0f, 0.02f) < 0.5f);
}

static void push_node(std::vector<uint8_t>& s, bool start, uint8_t quality, float angle_deg, float distance_mm) {
    const uint16_t angle_q6 = static_cast<uint16_t>(angle_deg * 64.0f);
    const uint16_t dist_q2 = static_cast<uint16_t>(distance_mm * 4.0f);
    s.push_back(static_cast<uint8_t>((quality << 2) | (start ? 0x01 : 0x02)));
    s.push_back(static_cast<uint8_t>(((angle_q6 << 1) & 0xFF) | 0x01));
    s.push_back(static_cast<uint8_t>(angle_q6 >> 7));
    s.push_back(static_cast<uint8_t>(dist_q2 & 0xFF));
    s.push_back(static_cast<uint8_t>(dist_q2 >> 8));
}

static void test_rplidar_bins_a_revolution() {
    std::vector<uint8_t> s = {0x12, 0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};  // junk + descriptor
    for (int rev = 0; rev < 2; rev++) {
        for (int i = 0; i < 400; i++) {
            const float angle = i * 0.9f;
            push_node(s, i == 0, 15, angle, i == 100 ? 0.0f : 1000.0f + angle);  // one dropout
        }
    }
    push_node(s, true, 15, 0.0f, 500.0f);

    RPLidar lidar;
    lidar.reset();
    int revolutions = 0;
    for (uint8_t b : s) {
        revolutions += lidar.feed(b);
    }
    TEST_ASSERT_EQUAL(3, revolutions);  // first start flag + two finished revolutions
    TEST_ASSERT_EQUAL(400, lidar.scan_points());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, lidar.scan()[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0909f, lidar.scan()[91]);  // nearest bin to 90.9 deg
    TEST_ASSERT_TRUE(std::isinf(lidar.scan()[90]));               // 90 deg had no valid echo
}

static void test_rplidar_resyncs_after_garbage() {
    std::vector<uint8_t> s = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81, 0x00, 0xFE};  // descriptor + 2 stray bytes
    for (int rev = 0; rev < 2; rev++) {
        for (int i = 0; i < 200; i++) {
            push_node(s, i == 0, 15, i * 1.8f, 2000.0f);
        }
    }
    push_node(s, true, 15, 0.0f, 2000.0f);

    RPLidar lidar;
    int revolutions = 0;
    for (uint8_t b : s) {
        revolutions += lidar.feed(b);
    }
    TEST_ASSERT_TRUE(revolutions >= 2);
    TEST_ASSERT_EQUAL(200, lidar.scan_points());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, lidar.scan()[180]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_forward_drive_turns_all_wheels_forward);
    RUN_TEST(test_ccw_rotation_spins_right_side_forward);
    RUN_TEST(test_strafe_left_pattern);
    RUN_TEST(test_forward_inverts_inverse);
    RUN_TEST(test_pid_tracks_setpoint_on_simulated_motor);
    RUN_TEST(test_pid_output_is_clamped_and_does_not_wind_up);
    RUN_TEST(test_rplidar_bins_a_revolution);
    RUN_TEST(test_rplidar_resyncs_after_garbage);
    return UNITY_END();
}
