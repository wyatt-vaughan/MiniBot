#ifndef __MMC5633_H__
#define __MMC5633_H__

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class MMC5633NJL {
public:
    static constexpr uint8_t I2C_ADDR = 0x30;

    static constexpr uint8_t REG_XOUT0 = 0x00;
    static constexpr uint8_t REG_PRODUCT_ID = 0x39;
    static constexpr uint8_t REG_STATUS1 = 0x18;
    static constexpr uint8_t REG_ODR = 0x1A;
    static constexpr uint8_t REG_CTRL0 = 0x1B;
    static constexpr uint8_t REG_CTRL1 = 0x1C;
    static constexpr uint8_t REG_CTRL2 = 0x1D;
    static constexpr uint8_t REG_ST_X_TH = 0x1E;
    static constexpr uint8_t REG_ST_Y_TH = 0x1F;
    static constexpr uint8_t REG_ST_Z_TH = 0x20;
    static constexpr uint8_t REG_ST_X = 0x27;
    static constexpr uint8_t REG_ST_Y = 0x28;
    static constexpr uint8_t REG_ST_Z = 0x29;

    static constexpr uint8_t STAT1_MEAS_M_DONE = (1 << 6);
    static constexpr uint8_t STAT1_SAT_SENSOR  = (1 << 5);

    static constexpr int32_t NULL_VALUE_20BIT = 524288;
    static constexpr float COUNTS_PER_G_20BIT = 16384.0f;

    uint32_t rawX = 0, rawY = 0, rawZ = 0;
    int8_t rawTemp = 0;

    MMC5633NJL(TwoWire &wire = Wire) : _wire(wire), _continuous_mode(false) {}

    bool begin(int sda_pin = -1, int scl_pin = -1, uint32_t i2c_freq = 400000) {
        if (sda_pin >= 0 && scl_pin >= 0) _wire.begin(sda_pin, scl_pin, i2c_freq);
        else _wire.begin();

        delay(10);

        uint8_t pid = 0;
        if (!readRegister(REG_PRODUCT_ID, &pid)) return false;
        if (pid != 0x10) return false;

        if (!writeRegister(REG_CTRL1, 0x03)) return false;
        if (!writeRegister(REG_ODR, 0xFF)) return false;
        if (!writeRegister(REG_CTRL0, 0xA0)) return false;

        disableContinuousMode();

        return true;
    }

    bool readMeasurement(uint32_t timeout_ms = 20) {
        if (_continuous_mode) {
            // In continuous mode, just read the latest data
            uint8_t buf[9] = {0};
            if (!readRegisters(REG_XOUT0, buf, 9)) return false;
            unpackRawXYZFromBuffer(buf, 9);
            rawTemp = buf[8];
            return true;
        } else {
            // On-demand mode: trigger measurement and wait
            uint8_t ctrl0 = (1 << 5) | (1 << 0);
            if (!writeRegister(REG_CTRL0, ctrl0)) return false;

            if (!waitForMeasurementDone(timeout_ms)) return false;

            uint8_t buf[9] = {0};
            if (!readRegisters(REG_XOUT0, buf, 9)) return false;

            unpackRawXYZFromBuffer(buf, 9);
            rawTemp = buf[8];
            return true;
        }
    }

    bool enableContinuousMode() {
        // Enable continuous measurement mode at 1000hz sampling
        if (!writeRegister(REG_CTRL2, 0x90)) return false;
        
        _continuous_mode = true;
        return true;
    }

    bool disableContinuousMode() {
        // Disable continuous measurement mode
        // TODO untested
        if (!writeRegister(REG_CTRL2, 0x00)) return false;
        _continuous_mode = false;
        return true;
    }

    bool runSelfTest(uint32_t timeout_ms = 100) {
        uint8_t stx = 0, sty = 0, stz = 0;
        if (!readRegister(REG_ST_X, &stx)) return false;
        if (!readRegister(REG_ST_Y, &sty)) return false;
        if (!readRegister(REG_ST_Z, &stz)) return false;

        uint8_t thx = (uint8_t)max(0, (int)round(stx * 0.8f));
        uint8_t thy = (uint8_t)max(0, (int)round(sty * 0.8f));
        uint8_t thz = (uint8_t)max(0, (int)round(stz * 0.8f));

        if (!writeRegister(REG_ST_X_TH, thx)) return false;
        if (!writeRegister(REG_ST_Y_TH, thy)) return false;
        if (!writeRegister(REG_ST_Z_TH, thz)) return false;

        if (!writeRegister(REG_CTRL0, 0x41)) return false;

        uint32_t start = millis();
        while (millis() - start < timeout_ms) {
            uint8_t status = 0;
            if (!readRegister(REG_STATUS1, &status)) return false;

            if ((status & STAT1_SAT_SENSOR) == 0) {
                return true;
            }
            delay(1);
        }

        return false;
    }

    int32_t signedX() const { return int32_t((int64_t)rawX - NULL_VALUE_20BIT); }
    int32_t signedY() const { return int32_t((int64_t)rawY - NULL_VALUE_20BIT); }
    int32_t signedZ() const { return int32_t((int64_t)rawZ - NULL_VALUE_20BIT); }

    float getFieldGaussX() const { return ((float) signedX()) / COUNTS_PER_G_20BIT; }
    float getFieldGaussY() const { return ((float) signedY()) / COUNTS_PER_G_20BIT; }
    float getFieldGaussZ() const { return ((float) signedZ()) / COUNTS_PER_G_20BIT; }

    float getAzimuthDegrees() const {
        float fx = (float) signedX();
        float fy = (float) signedY();
        float ang = atan2f(fy, fx);
        float deg = ang * 180.0f / M_PI;
        return deg;
    }

    float getAzimuthRadians() const {
        float fx = (float) signedX();
        float fy = (float) signedY();
        return atan2f(fy, fx);
    }

private:
    TwoWire &_wire;
    bool _continuous_mode;

    bool waitForMeasurementDone(uint32_t timeout_ms) {
        uint32_t start = millis();
        while (millis() - start < timeout_ms) {
            uint8_t status = 0;
            if (!readRegister(REG_STATUS1, &status)) return false;
            if (status & STAT1_MEAS_M_DONE) return true;
        }
        return false;
    }

    void unpackRawXYZFromBuffer(const uint8_t *buf, size_t len) {
        if (len < 9) return;

        uint32_t x_hi = ((uint32_t)buf[0] << 12);
        uint32_t x_mid = ((uint32_t)buf[1] << 4);
        uint32_t x_lo = ((uint32_t)buf[6] >> 4) & 0x0F;
        rawX = x_hi | x_mid | x_lo;

        uint32_t y_hi = ((uint32_t)buf[2] << 12);
        uint32_t y_mid = ((uint32_t)buf[3] << 4);
        uint32_t y_lo = ((uint32_t)buf[6] & 0x0F);
        rawY = y_hi | y_mid | y_lo;

        uint32_t z_hi = ((uint32_t)buf[4] << 12);
        uint32_t z_mid = ((uint32_t)buf[5] << 4);
        uint32_t z_lo = ((uint32_t)buf[7] >> 4) & 0x0F;
        rawZ = z_hi | z_mid | z_lo;
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
        _wire.beginTransmission(I2C_ADDR);
        _wire.write(reg);
        _wire.write(value);
        return (_wire.endTransmission() == 0);
    }

    bool readRegister(uint8_t reg, uint8_t *value) {
        _wire.beginTransmission(I2C_ADDR);
        _wire.write(reg);
        if (_wire.endTransmission(false) != 0) return false;
        if (_wire.requestFrom(I2C_ADDR, (uint8_t)1) != 1) return false;
        *value = _wire.read();
        return true;
    }

    bool readRegisters(uint8_t reg, uint8_t *buf, size_t len) {
        _wire.beginTransmission(I2C_ADDR);
        _wire.write(reg);
        if (_wire.endTransmission(false) != 0) return false;
        if (_wire.requestFrom(I2C_ADDR, (uint8_t)len) != len) return false;
        for (size_t i = 0; i < len; ++i) buf[i] = _wire.read();
        return true;
    }
};

#endif // __MMC5633_H__
