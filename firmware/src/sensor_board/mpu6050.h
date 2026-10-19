// Bare-bones MPU6050 (GY-521) driver: +-4 g, +-500 deg/s, 44 Hz low pass.
#pragma once

#include <Arduino.h>
#include <Wire.h>

class Mpu6050 {
public:
    struct Sample {
        float ax, ay, az;  // m/s^2
        float gx, gy, gz;  // rad/s
        float temperature; // degC
    };

    bool begin(TwoWire& wire, uint8_t address) {
        wire_ = &wire;
        address_ = address;
        return write(0x6B, 0x01)     // PWR_MGMT_1: wake up, gyro PLL clock
               && write(0x1A, 0x03)  // CONFIG: DLPF 44 Hz
               && write(0x19, 0x04)  // SMPLRT_DIV: 200 Hz
               && write(0x1B, 0x08)  // GYRO_CONFIG: +-500 deg/s
               && write(0x1C, 0x08); // ACCEL_CONFIG: +-4 g
    }

    bool read(Sample& s) {
        uint8_t b[14];
        wire_->beginTransmission(address_);
        wire_->write(0x3B);  // ACCEL_XOUT_H, followed by temperature and gyro
        if (wire_->endTransmission(false) != 0 || wire_->requestFrom(address_, static_cast<uint8_t>(sizeof(b))) != sizeof(b)) {
            return false;
        }
        for (uint8_t& v : b) {
            v = wire_->read();
        }
        constexpr float ACCEL = 9.80665f / 8192.0f;
        constexpr float GYRO = (PI / 180.0f) / 65.5f;
        s.ax = word(b, 0) * ACCEL;
        s.ay = word(b, 2) * ACCEL;
        s.az = word(b, 4) * ACCEL;
        s.temperature = word(b, 6) / 340.0f + 36.53f;
        s.gx = word(b, 8) * GYRO;
        s.gy = word(b, 10) * GYRO;
        s.gz = word(b, 12) * GYRO;
        return true;
    }

private:
    static int16_t word(const uint8_t* b, int i) { return static_cast<int16_t>((b[i] << 8) | b[i + 1]); }

    bool write(uint8_t reg, uint8_t value) {
        wire_->beginTransmission(address_);
        wire_->write(reg);
        wire_->write(value);
        return wire_->endTransmission() == 0;
    }

    TwoWire* wire_ = nullptr;
    uint8_t address_ = 0;
};
