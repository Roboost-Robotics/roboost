// Sensor board: RPLidar A1 -> scan, MPU6050 -> imu/data_raw + imu/temperature,
// battery divider -> battery. Everything is published best effort.
#include <Arduino.h>
#include <Wire.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/temperature.h>

#include "../common/uros.h"
#include "config.h"
#include "mpu6050.h"
#include "rplidar.h"

namespace {

HardwareSerial& lidar_serial = Serial2;
RPLidar lidar;
int64_t revolution_start_ns = 0;
uint32_t last_revolution_ms = 0;

Mpu6050 imu;
bool imu_ok = false;
float gyro_bias[3] = {};

rcl_publisher_t scan_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t temperature_pub;
rcl_publisher_t battery_pub;
sensor_msgs__msg__LaserScan scan_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Temperature temperature_msg;
sensor_msgs__msg__BatteryState battery_msg;

// ---- Lidar ------------------------------------------------------------------

void lidar_start() {
    digitalWrite(config::LIDAR_MOTOR, HIGH);
    lidar_serial.write(RPLidar::CMD_STOP, 2);
    delay(10);
    while (lidar_serial.available()) {
        lidar_serial.read();
    }
    lidar.reset();
    lidar_serial.write(RPLidar::CMD_SCAN, 2);
    revolution_start_ns = 0;
    last_revolution_ms = millis();
}

void lidar_stop() {
    lidar_serial.write(RPLidar::CMD_STOP, 2);
    digitalWrite(config::LIDAR_MOTOR, LOW);
}

void process_lidar(bool connected) {
    for (int n = lidar_serial.available(); n > 0; n--) {
        if (!lidar.feed(lidar_serial.read())) {
            continue;
        }
        // A revolution just finished. Stamp = its first beam, as LaserScan expects.
        last_revolution_ms = millis();
        if (!connected) {
            continue;
        }
        const int64_t now = uros::now_ns();
        const int64_t duration = now - revolution_start_ns;
        if (revolution_start_ns > 0 && duration < 1000000000LL && lidar.scan_points() >= config::LIDAR_MIN_POINTS) {
            uros::stamp(scan_msg.header.stamp, revolution_start_ns);
            scan_msg.scan_time = duration * 1e-9f;
            scan_msg.time_increment = scan_msg.scan_time / RPLidar::BEAMS;
            memcpy(scan_msg.ranges.data, lidar.scan(), sizeof(float) * RPLidar::BEAMS);
            uros::ignore(rcl_publish(&scan_pub, &scan_msg, nullptr));
        }
        revolution_start_ns = now;
    }
    if (connected && millis() - last_revolution_ms > 3000) {
        Serial.println("[sensor_board] no lidar data, restarting scan");
        lidar_start();
    }
}

// ---- IMU --------------------------------------------------------------------

bool imu_begin() {
    if (!imu.begin(Wire, config::IMU_ADDRESS)) {
        return false;
    }
    // Gyro bias: the robot is expected to stand still for this second.
    float sum[3] = {};
    int n = 0;
    for (int i = 0; i < 200; i++) {
        Mpu6050::Sample s;
        if (imu.read(s)) {
            sum[0] += s.gx;
            sum[1] += s.gy;
            sum[2] += s.gz;
            n++;
        }
        delay(5);
    }
    for (int i = 0; i < 3 && n > 0; i++) {
        gyro_bias[i] = sum[i] / n;
    }
    return n > 0;
}

void publish_imu() {
    Mpu6050::Sample s;
    if (!imu.read(s)) {
        imu_ok = false;
        return;
    }
    uros::stamp(imu_msg.header.stamp);
    imu_msg.angular_velocity.x = s.gx - gyro_bias[0];
    imu_msg.angular_velocity.y = s.gy - gyro_bias[1];
    imu_msg.angular_velocity.z = s.gz - gyro_bias[2];
    imu_msg.linear_acceleration.x = s.ax;
    imu_msg.linear_acceleration.y = s.ay;
    imu_msg.linear_acceleration.z = s.az;
    uros::ignore(rcl_publish(&imu_pub, &imu_msg, nullptr));

    temperature_msg.header.stamp = imu_msg.header.stamp;
    temperature_msg.temperature = s.temperature;
}

// ---- Battery ----------------------------------------------------------------

float battery_voltage() {
    uint32_t mv = 0;
    for (int i = 0; i < 16; i++) {
        mv += analogReadMilliVolts(config::BATTERY_PIN);
    }
    return mv / 16.0f / 1000.0f * config::BATTERY_DIVIDER;
}

// ---- micro-ROS ----------------------------------------------------------------

bool create() {
    scan_pub = rcl_get_zero_initialized_publisher();
    imu_pub = rcl_get_zero_initialized_publisher();
    temperature_pub = rcl_get_zero_initialized_publisher();
    battery_pub = rcl_get_zero_initialized_publisher();
    const bool ok =
        rclc_publisher_init_best_effort(&scan_pub, &uros::node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan") == RCL_RET_OK &&
        rclc_publisher_init_best_effort(&imu_pub, &uros::node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw") == RCL_RET_OK &&
        rclc_publisher_init_best_effort(&temperature_pub, &uros::node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "imu/temperature") == RCL_RET_OK &&
        rclc_publisher_init_best_effort(&battery_pub, &uros::node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "battery") == RCL_RET_OK;
    if (ok) {
        lidar_start();  // only spin the lidar while someone can receive the data
    }
    return ok;
}

void destroy() {
    lidar_stop();
    uros::ignore(rcl_publisher_fini(&scan_pub, &uros::node));
    uros::ignore(rcl_publisher_fini(&imu_pub, &uros::node));
    uros::ignore(rcl_publisher_fini(&temperature_pub, &uros::node));
    uros::ignore(rcl_publisher_fini(&battery_pub, &uros::node));
}

void init_messages() {
    sensor_msgs__msg__LaserScan__init(&scan_msg);
    rosidl_runtime_c__String__assign(&scan_msg.header.frame_id, "lidar_link");
    // RPLidar angles run clockwise; lidar_link is mounted upside down in the URDF to match.
    scan_msg.angle_min = 0.0f;
    scan_msg.angle_increment = 2.0f * PI / RPLidar::BEAMS;
    scan_msg.angle_max = scan_msg.angle_increment * (RPLidar::BEAMS - 1);
    scan_msg.range_min = 0.15f;
    scan_msg.range_max = 12.0f;
    rosidl_runtime_c__float__Sequence__init(&scan_msg.ranges, RPLidar::BEAMS);

    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
    imu_msg.orientation_covariance[0] = -1.0;  // no orientation estimate
    for (int i = 0; i < 3; i++) {
        imu_msg.angular_velocity_covariance[i * 4] = 2.5e-5;
        imu_msg.linear_acceleration_covariance[i * 4] = 2.5e-3;
    }

    sensor_msgs__msg__Temperature__init(&temperature_msg);
    rosidl_runtime_c__String__assign(&temperature_msg.header.frame_id, "imu_link");

    sensor_msgs__msg__BatteryState__init(&battery_msg);
    battery_msg.temperature = NAN;
    battery_msg.current = NAN;
    battery_msg.charge = NAN;
    battery_msg.capacity = NAN;
    battery_msg.design_capacity = NAN;
    battery_msg.percentage = NAN;
    battery_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
}

}  // namespace

void setup() {
    Serial.begin(115200);
    pinMode(config::LIDAR_MOTOR, OUTPUT);
    digitalWrite(config::LIDAR_MOTOR, LOW);
    pinMode(config::LOW_BATTERY_LED, OUTPUT);
    lidar_serial.setRxBufferSize(4096);  // ~350 ms of lidar data, covers WiFi hiccups
    lidar_serial.begin(115200, SERIAL_8N1, config::LIDAR_RX, config::LIDAR_TX);
    lidar_serial.write(RPLidar::CMD_STOP, 2);  // it may still be scanning from before a reset

    Wire.begin(config::IMU_SDA, config::IMU_SCL, 400000);
    imu_ok = imu_begin();
    if (!imu_ok) {
        Serial.println("[sensor_board] MPU6050 not found, retrying in background");
    }
    init_messages();

    uros::begin("sensor_board", config::STATUS_LED);
}

void loop() {
    static uint32_t last_imu_ms = 0;
    static uint32_t last_battery_ms = 0;
    const uint32_t now = millis();
    const bool connected = uros::spin(create, destroy);

    process_lidar(connected);

    if (now - last_imu_ms >= config::IMU_PERIOD_MS) {
        last_imu_ms = now;
        if (!imu_ok && now % 2000 < config::IMU_PERIOD_MS) {
            imu_ok = imu_begin();
        }
        if (imu_ok && connected) {
            publish_imu();
        }
    }

    if (now - last_battery_ms >= config::BATTERY_PERIOD_MS) {
        last_battery_ms = now;
        const float voltage = battery_voltage();
        const bool present = voltage > 5.0f;  // below that we are on USB power
        digitalWrite(config::LOW_BATTERY_LED, present && voltage < config::LOW_BATTERY_VOLTAGE);
        if (connected) {
            uros::stamp(battery_msg.header.stamp);
            battery_msg.voltage = voltage;
            battery_msg.present = present;
            uros::ignore(rcl_publish(&battery_pub, &battery_msg, nullptr));
            if (imu_ok) {
                uros::ignore(rcl_publish(&temperature_pub, &temperature_msg, nullptr));
            }
        }
    }
}
