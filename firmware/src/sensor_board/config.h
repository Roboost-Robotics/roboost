// Sensor board: ESP32 DevKit V1 + RPLidar A1 + MPU6050 (GY-521) + battery voltage divider.
#pragma once

#include <stdint.h>

namespace config {

constexpr int STATUS_LED = 2;

// RPLidar A1: UART2 through the level shifter (U3), motor enable on MTC (U2).
constexpr int LIDAR_RX = 16;
constexpr int LIDAR_TX = 17;
constexpr int LIDAR_MOTOR = 18;
constexpr int LIDAR_MIN_POINTS = 100;  // drop partial revolutions (motor spin-up, data loss)

// MPU6050 on the I2C header pins.
constexpr int IMU_SDA = 21;
constexpr int IMU_SCL = 22;
constexpr uint8_t IMU_ADDRESS = 0x68;
constexpr uint32_t IMU_PERIOD_MS = 20;  // 50 Hz

// Battery: divider on P1 -> GPIO36, red LED1 on GPIO13.
constexpr int BATTERY_PIN = 36;
constexpr int LOW_BATTERY_LED = 13;
constexpr float BATTERY_DIVIDER = 4.09f;      // V_battery / V_pin, check against a multimeter
constexpr float LOW_BATTERY_VOLTAGE = 10.5f;  // 3S LiPo
constexpr uint32_t BATTERY_PERIOD_MS = 1000;

}  // namespace config
