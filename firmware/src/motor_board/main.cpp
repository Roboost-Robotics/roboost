// Motor board: cmd_vel -> mecanum inverse kinematics -> one velocity PID per wheel.
// Publishes joint_states (wheel angle, speed, duty as effort) and wheel_odom.
#include <Arduino.h>
#include <ESP32Encoder.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/joint_state.h>

#include "../common/uros.h"
#include "config.h"
#include "mecanum.h"
#include "pid.h"

namespace {

const Mecanum kinematics(config::WHEEL_RADIUS, config::WHEEL_BASE, config::TRACK_WIDTH);
ESP32Encoder encoders[4];
Pid pids[4] = {Pid(config::PID), Pid(config::PID), Pid(config::PID), Pid(config::PID)};
float rad_per_count[4];

// Shared between the control task and loop().
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
Twist2D command = {0, 0, 0};
uint32_t command_ms = 0;
struct State {
    float angle[4];  // rad
    float speed[4];  // rad/s, filtered
    float duty[4];   // -1..1
    float x, y, yaw;
    Twist2D twist;
} state = {};

rcl_subscription_t cmd_vel_sub;
rcl_publisher_t joint_pub;
rcl_publisher_t odom_pub;
rclc_executor_t executor;
geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__JointState joint_msg;
nav_msgs__msg__Odometry odom_msg;

void set_motor(int i, float duty) {
    const config::Wheel& w = config::WHEELS[i];
    if (w.invert) {
        duty = -duty;
    }
    digitalWrite(w.in1, duty > 0.0f ? HIGH : LOW);
    digitalWrite(w.in2, duty < 0.0f ? HIGH : LOW);
    ledcWrite(w.pwm_channel, static_cast<uint32_t>(fabsf(duty) * ((1u << config::PWM_BITS) - 1)));
}

void control_task(void*) {
    const float dt = config::CONTROL_PERIOD_MS / 1000.0f;
    int64_t last_count[4] = {};
    float speed[4] = {};
    float setpoint[4] = {};
    float x = 0.0f, y = 0.0f, yaw = 0.0f;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(config::CONTROL_PERIOD_MS));

        float angle[4];
        float raw_speed[4];
        for (int i = 0; i < 4; i++) {
            int64_t count = encoders[i].getCount();
            if (config::WHEELS[i].invert) {
                count = -count;
            }
            angle[i] = count * rad_per_count[i];
            raw_speed[i] = (count - last_count[i]) * rad_per_count[i] / dt;
            last_count[i] = count;
            speed[i] += config::VELOCITY_FILTER * (raw_speed[i] - speed[i]);
        }

        Twist2D cmd;
        uint32_t cmd_ms;
        portENTER_CRITICAL(&lock);
        cmd = command;
        cmd_ms = command_ms;
        portEXIT_CRITICAL(&lock);
        if (millis() - cmd_ms > config::CMD_VEL_TIMEOUT_MS) {
            cmd = {0.0f, 0.0f, 0.0f};
        }

        // Wheel setpoints: scale down together (keeps the direction), then ramp.
        float target[4];
        kinematics.inverse(cmd, target);
        float peak = 0.0f;
        for (float t : target) {
            peak = fmaxf(peak, fabsf(t));
        }
        const float scale = peak > config::MAX_WHEEL_SPEED ? config::MAX_WHEEL_SPEED / peak : 1.0f;
        const float max_step = config::MAX_WHEEL_ACCEL * dt;
        bool idle = true;
        for (int i = 0; i < 4; i++) {
            setpoint[i] += constrain(target[i] * scale - setpoint[i], -max_step, max_step);
            idle = idle && setpoint[i] == 0.0f;
        }

        float duty[4];
        for (int i = 0; i < 4; i++) {
            if (idle) {
                pids[i].reset();  // standing still: motors off instead of fighting encoder noise
                duty[i] = 0.0f;
            } else {
                duty[i] = pids[i].update(setpoint[i], speed[i], dt);
            }
            set_motor(i, duty[i]);
        }

        // Odometry from the unfiltered wheel increments (midpoint integration).
        const Twist2D v = kinematics.forward(raw_speed);
        const float heading = yaw + 0.5f * v.wz * dt;
        x += (v.vx * cosf(heading) - v.vy * sinf(heading)) * dt;
        y += (v.vx * sinf(heading) + v.vy * cosf(heading)) * dt;
        yaw = atan2f(sinf(yaw + v.wz * dt), cosf(yaw + v.wz * dt));

        const Twist2D twist = kinematics.forward(speed);
        portENTER_CRITICAL(&lock);
        for (int i = 0; i < 4; i++) {
            state.angle[i] = angle[i];
            state.speed[i] = speed[i];
            state.duty[i] = duty[i];
        }
        state.x = x;
        state.y = y;
        state.yaw = yaw;
        state.twist = twist;
        portEXIT_CRITICAL(&lock);
    }
}

void on_cmd_vel(const void* msg) {
    const auto* twist = static_cast<const geometry_msgs__msg__Twist*>(msg);
    portENTER_CRITICAL(&lock);
    command = {static_cast<float>(twist->linear.x), static_cast<float>(twist->linear.y), static_cast<float>(twist->angular.z)};
    command_ms = millis();
    portEXIT_CRITICAL(&lock);
}

// All topics are best effort: a late wheel state is useless, and reliable publishing
// blocks until the agent acknowledges.
bool create() {
    cmd_vel_sub = rcl_get_zero_initialized_subscription();
    joint_pub = rcl_get_zero_initialized_publisher();
    odom_pub = rcl_get_zero_initialized_publisher();
    executor = rclc_executor_get_zero_initialized_executor();
    return rclc_subscription_init_best_effort(&cmd_vel_sub, &uros::node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") == RCL_RET_OK &&
           rclc_publisher_init_best_effort(&joint_pub, &uros::node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states") == RCL_RET_OK &&
           rclc_publisher_init_best_effort(&odom_pub, &uros::node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "wheel_odom") == RCL_RET_OK &&
           rclc_executor_init(&executor, &uros::support.context, 1, &uros::allocator) == RCL_RET_OK &&
           rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &on_cmd_vel, ON_NEW_DATA) == RCL_RET_OK;
}

void destroy() {
    uros::ignore(rclc_executor_fini(&executor));
    uros::ignore(rcl_subscription_fini(&cmd_vel_sub, &uros::node));
    uros::ignore(rcl_publisher_fini(&joint_pub, &uros::node));
    uros::ignore(rcl_publisher_fini(&odom_pub, &uros::node));
}

void init_messages() {
    sensor_msgs__msg__JointState__init(&joint_msg);
    rosidl_runtime_c__String__Sequence__init(&joint_msg.name, 4);
    for (int i = 0; i < 4; i++) {
        rosidl_runtime_c__String__assign(&joint_msg.name.data[i], config::WHEELS[i].joint);
    }
    rosidl_runtime_c__double__Sequence__init(&joint_msg.position, 4);
    rosidl_runtime_c__double__Sequence__init(&joint_msg.velocity, 4);
    rosidl_runtime_c__double__Sequence__init(&joint_msg.effort, 4);

    nav_msgs__msg__Odometry__init(&odom_msg);
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_footprint");
    // x, y, z, roll, pitch, yaw. Planar robot: z/roll/pitch are unknown.
    const double pose_cov[6] = {0.01, 0.01, 1e6, 1e6, 1e6, 0.05};
    const double twist_cov[6] = {0.005, 0.005, 1e6, 1e6, 1e6, 0.02};
    for (int i = 0; i < 6; i++) {
        odom_msg.pose.covariance[i * 7] = pose_cov[i];
        odom_msg.twist.covariance[i * 7] = twist_cov[i];
    }
}

void publish() {
    State s;
    portENTER_CRITICAL(&lock);
    s = state;
    portEXIT_CRITICAL(&lock);

    uros::stamp(joint_msg.header.stamp);
    for (int i = 0; i < 4; i++) {
        joint_msg.position.data[i] = s.angle[i];
        joint_msg.velocity.data[i] = s.speed[i];
        joint_msg.effort.data[i] = s.duty[i];
    }
    uros::ignore(rcl_publish(&joint_pub, &joint_msg, nullptr));

    odom_msg.header.stamp = joint_msg.header.stamp;
    odom_msg.pose.pose.position.x = s.x;
    odom_msg.pose.pose.position.y = s.y;
    odom_msg.pose.pose.orientation.z = sinf(s.yaw / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(s.yaw / 2.0f);
    odom_msg.twist.twist.linear.x = s.twist.vx;
    odom_msg.twist.twist.linear.y = s.twist.vy;
    odom_msg.twist.twist.angular.z = s.twist.wz;
    uros::ignore(rcl_publish(&odom_pub, &odom_msg, nullptr));
}

}  // namespace

void setup() {
    Serial.begin(115200);

    ESP32Encoder::useInternalWeakPullResistors = puType::down;
    for (int i = 0; i < 4; i++) {
        const config::Wheel& w = config::WHEELS[i];
        pinMode(w.in1, OUTPUT);
        pinMode(w.in2, OUTPUT);
        ledcSetup(w.pwm_channel, config::PWM_FREQUENCY, config::PWM_BITS);
        ledcAttachPin(w.ena, w.pwm_channel);
        set_motor(i, 0.0f);
        encoders[i].attachFullQuad(w.enc_a, w.enc_b);
        rad_per_count[i] = 2.0f * PI / (4.0f * w.encoder_ppr);
    }
    init_messages();

    // Above loop() priority on the same core; WiFi runs on core 0.
    xTaskCreatePinnedToCore(control_task, "control", 4096, nullptr, 5, nullptr, 1);

    uros::begin("motor_board", config::STATUS_LED);
}

void loop() {
    static uint32_t last_publish_ms = 0;
    if (!uros::spin(create, destroy)) {
        delay(10);
        return;
    }
    uros::ignore(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
    const uint32_t now = millis();
    if (now - last_publish_ms >= config::PUBLISH_PERIOD_MS) {
        last_publish_ms = now - (now - last_publish_ms) % config::PUBLISH_PERIOD_MS;  // stay on the 20 Hz grid
        publish();
    }
}
