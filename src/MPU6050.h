// MPU6050.h
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class MPU6050 {
public:
    /// all accessible sensor outputs in one struct
    struct Data {
      // raw (converted) readings
      float accX, accY, accZ;       // in g
      float gyroRateX, gyroRateY, gyroRateZ; // in °/s
      float temperature;            // °C

      // computed tilt angles from accel
      float accAngleX, accAngleY;
      // integrated angles from gyro
      float gyroRateX_dt, gyroRateY_dt;
    } data;


    MPU6050(uint8_t address = 0x68)
      : _address(address), _prevTime(0)
    {}

    /// begin I2C, wake sensor
    void init(int sdaPin, int sclPin, uint32_t baud = 115200) {
      Serial.begin(baud);
      while (!Serial) delay(10);
      Wire.begin(sdaPin, sclPin);
      Wire.beginTransmission(_address);
      Wire.write(0x6B); // Power management register
      Wire.write(0x00);   // wake up MPU-6050
      Wire.endTransmission(true); // end transmission to wake up MPU-6050
      // set up initial timestamp
      _prevTime = micros();
      // zero-out integrated angles
      data.gyroRateX_dt = data.gyroRateY_dt = 0.0f;
      
    }

    /// call this once per loop to update all fields in `data`
    void readSensorData() {
        unsigned long now = micros();
        float dt = (now - _prevTime) * 1e-6f;
        _prevTime = now;
        if (dt <= 0) return;

        // request 14 bytes starting at ACCEL_XOUT_H
        Wire.beginTransmission(_address);
        Wire.write(0x3B);
        Wire.endTransmission(false); // false means "do not release bus"
        Wire.requestFrom(_address, 14, true);

        // explicit 16-bit reads:
        int16_t rawAx = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t rawAy = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t rawAz = (int16_t)((Wire.read() << 8) | Wire.read());

        int16_t rawT  = (int16_t)((Wire.read() << 8) | Wire.read());

        int16_t rawGx = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t rawGy = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t rawGz = (int16_t)((Wire.read() << 8) | Wire.read());

        // convert:
        data.accX = rawAx / 16384.0f;
        data.accY = rawAy / 16384.0f;
        data.accZ = rawAz / 16384.0f;
        data.temperature = rawT / 340.0f + 36.53f;
        data.gyroRateX = (rawGx / 131.0f); 
        data.gyroRateY = (rawGy / 131.0f);
        data.gyroRateZ = (rawGz / 131.0f);
    
        // angles:
        data.accAngleX = atan2f(data.accY, data.accZ)  * RAD_TO_DEG;
        data.accAngleY = atan2f(-data.accX,
                            sqrtf(data.accY*data.accY + data.accZ*data.accZ)) * RAD_TO_DEG;

        // integrate to get momentary change in angle
        data.gyroRateX_dt = data.gyroRateX * dt;
        data.gyroRateY_dt = data.gyroRateY * dt;
        
        
    }

private:
    uint8_t  _address;
    unsigned long _prevTime;


};
