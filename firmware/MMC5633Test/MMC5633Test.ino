#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class MMC5633NJL {
public:
    // I2C
    static constexpr uint8_t I2C_ADDR = 0x30;

    // Registers
    static constexpr uint8_t REG_XOUT0 = 0x00;
    static constexpr uint8_t REG_XOUT1 = 0x01;
    static constexpr uint8_t REG_YOUT0 = 0x02;
    static constexpr uint8_t REG_YOUT1 = 0x03;
    static constexpr uint8_t REG_ZOUT0 = 0x04;
    static constexpr uint8_t REG_ZOUT1 = 0x05;
    static constexpr uint8_t REG_XOUT2 = 0x06; // Xout2, Yout2, Zout2 at 06h,07h,08h
    static constexpr uint8_t REG_XOUT2_ADDR = 0x06;
    static constexpr uint8_t REG_YOUT2_ADDR = 0x07;
    static constexpr uint8_t REG_ZOUT2_ADDR = 0x08;

    static constexpr uint8_t REG_TEMP = 0x09;
    static constexpr uint8_t REG_TPH0 = 0x0A;
    static constexpr uint8_t REG_TPH1 = 0x0B;
    static constexpr uint8_t REG_TU   = 0x0C;

    static constexpr uint8_t REG_STATUS1 = 0x18;
    static constexpr uint8_t REG_STATUS0 = 0x19;

    static constexpr uint8_t REG_ODR = 0x1A;
    static constexpr uint8_t REG_CTRL0 = 0x1B; // Internal Control 0
    static constexpr uint8_t REG_CTRL1 = 0x1C; // Internal Control 1
    static constexpr uint8_t REG_CTRL2 = 0x1D; // Internal Control 2

    static constexpr uint8_t REG_ST_X_TH = 0x1E;
    static constexpr uint8_t REG_ST_Y_TH = 0x1F;
    static constexpr uint8_t REG_ST_Z_TH = 0x20;

    static constexpr uint8_t REG_ST_X = 0x27;
    static constexpr uint8_t REG_ST_Y = 0x28;
    static constexpr uint8_t REG_ST_Z = 0x29;

    static constexpr uint8_t REG_PRODUCT_ID = 0x39;

    // Status1 bits (from datasheet register bit mapping)
    static constexpr uint8_t STAT1_MEAS_M_DONE = (1 << 6); // bit6 = Meas_m_done
    static constexpr uint8_t STAT1_SAT_SENSOR  = (1 << 5); // bit5 = Sat_sensor (0 => PASS)

    // Conversion constants (20-bit mode)
    static constexpr int32_t NULL_VALUE_20BIT = 524288; // mid-scale null-field counts for 20-bit
    static constexpr float COUNTS_PER_G_20BIT = 16384.0f; // counts per Gauss (20-bit)

    // Public raw data (unsigned raw as read from device)
    uint32_t rawX = 0, rawY = 0, rawZ = 0;
    int8_t rawTemp = 0;

    // Constructor: optionally pass TwoWire instance (defaults to Wire)
    MMC5633NJL(TwoWire &wire = Wire) : _wire(wire) {}

    // Initialize I2C and basic recommended settings
    bool begin(int sda_pin = -1, int scl_pin = -1, uint32_t i2c_freq = 400000) {
        // begin Wire with optional custom pins (ESP32 Wire.begin(sda, scl))
        if (sda_pin >= 0 && scl_pin >= 0) _wire.begin(sda_pin, scl_pin, i2c_freq);
        else _wire.begin();

        // small delay for device power-up
        delay(10);

        // confirm product id (datasheet reset value: 0x10)
        uint8_t pid = 0;
        if (!readRegister(REG_PRODUCT_ID, &pid)) return false;
        if (pid != 0x10) {
            // Unexpected product id — still return false
            return false;
        }

        // Recommended: enable automatic set/reset (Auto_SR_en bit, bit5 of REG_CTRL0)
        // Build ctrl0 with Auto_SR_en = 1 (bit5). Note: other bits are write-only and may be used later.
        uint8_t ctrl0 = (1 << 5); // Auto_SR_en
        if (!writeRegister(REG_CTRL0, ctrl0)) return false;

        // Set default bandwidth to BW = 10 (BW1=1, BW0=0) -> faster measurement (2.0 ms)
        // REG_CTRL1 bits: BW1 (bit1), BW0 (bit0)
        uint8_t ctrl1 = (1 << 1); // BW1=1, BW0=0
        if (!writeRegister(REG_CTRL1, ctrl1)) return false;

        // Clear/control2 default (ensure continuous mode off)
        writeRegister(REG_CTRL2, 0x00);

        return true;
    }

    // Triggers a single magnetic measurement (and returns when ready) then reads data.
    bool readMeasurement(uint32_t timeout_ms = 20) {
        // Trigger magnetic measurement by writing Take_meas_M bit (bit0) in CTRL0.
        // But preserve Auto_SR_en (bit5) if we want it on (recommended).
        // We'll set Auto_SR_en=1 and set Take_meas_M=1.
        uint8_t ctrl0 = (1 << 5) | (1 << 0); // Auto_SR_en + Take_meas_M
        if (!writeRegister(REG_CTRL0, ctrl0)) return false;

        // Wait for Meas_m_done (poll STATUS1)
        if (!waitForMeasurementDone(timeout_ms)) return false;

        // Read XYZ registers: 9 bytes: Xout0, Xout1, Yout0, Yout1, Zout0, Zout1, Xout2, Yout2, Zout2
        uint8_t buf[9] = {0};
        if (!readRegisters(REG_XOUT0, buf, 9)) return false;

        unpackRawXYZFromBuffer(buf, 9);
        rawTemp = buf[8]; // temperature register is at offset 8 for full-read pattern (per datasheet order in RDLONG)
        return true;
    }

    // enableContinuous(odr): start continuous mode with ODR in Hz (1..255). For 1000Hz use odr=255 + hpower bit.
    bool enableContinuous(uint8_t odr, bool hpower_for_1000hz = false) {
        if ((odr <= 0) || (odr > 1000) || (255 < odr < 999)) return false; // must be between 1-255 or 1000
        if (!writeRegister(REG_ODR, odr)) return false;

        // Set Cmm_freq_en bit (bit7 of CTRL0) to let internal circuits compute counters.
        // Also ensure Auto_SR_en remains set (bit5). We'll set Auto_SR_en too.
        uint8_t ctrl0 = (1 << 7) | (1 << 5); // Cmm_freq_en + Auto_SR_en
        if (!writeRegister(REG_CTRL0, ctrl0)) return false;

        // Write CTRL2 to set Cmm_en bit (bit4) to enter continuous mode.
        // Also set hpower bit (bit7) if 1000Hz desired (and device supports it).
        uint8_t ctrl2 = 0;
        if (hpower_for_1000hz) ctrl2 |= (1 << 7); // hpower
        ctrl2 |= (1 << 4); // Cmm_en
        if (!writeRegister(REG_CTRL2, ctrl2)) return false;

        _continuous_enabled = true;
        return true;
    }

    // Turn continuous mode off.
    bool disableContinuous() {
        // Clear Cmm_en bit in CTRL2 (write zeros except bits we want default)
        // Since CTRL2 is write-only, write 0 to clear Cmm_en
        if (!writeRegister(REG_CTRL2, 0x00)) return false;

        // Set ODR to zero to avoid accidental continuous behavior
        writeRegister(REG_ODR, 0x00);

        _continuous_enabled = false;
        return true;
    }

    // Poll continuous: check status bit and read latest sample if available.
    // Returns true if a new sample was read (and internal rawX/Y/Z updated).
    bool pollContinuous() {
        if (!_continuous_enabled) return false;

        uint8_t status = 0;
        if (!readRegister(REG_STATUS1, &status)) return false;

        if (status & STAT1_MEAS_M_DONE) {
            // Read registers (same order as single-shot read)
            uint8_t buf[9];
            if (!readRegisters(REG_XOUT0, buf, 9)) return false;
            unpackRawXYZFromBuffer(buf, 9);
            rawTemp = buf[8];
            return true;
        }
        return false; // no new data
    }

    // Returns true on PASS, false on FAIL or error.
    bool runSelfTest(uint32_t timeout_ms = 100) {
        // 1) Read ST_X/ST_Y/ST_Z from registers 0x27..0x29
        uint8_t stx = 0, sty = 0, stz = 0;
        if (!readRegister(REG_ST_X, &stx)) return false;
        if (!readRegister(REG_ST_Y, &sty)) return false;
        if (!readRegister(REG_ST_Z, &stz)) return false;

        // 2) Calculate thresholds as 80% of those values (per datasheet)
        uint8_t thx = (uint8_t)max(0, (int)round(stx * 0.8f));
        uint8_t thy = (uint8_t)max(0, (int)round(sty * 0.8f));
        uint8_t thz = (uint8_t)max(0, (int)round(stz * 0.8f));

        // 3) Write thresholds into ST_X_TH (0x1E), ST_Y_TH (0x1F), ST_Z_TH (0x20)
        if (!writeRegister(REG_ST_X_TH, thx)) return false;
        if (!writeRegister(REG_ST_Y_TH, thy)) return false;
        if (!writeRegister(REG_ST_Z_TH, thz)) return false;

        // 4) Initiate self-test: Write 0x41 to CTRL0 (0100 0001 => Auto_st_en (bit6) + Take_meas_M (bit0))
        if (!writeRegister(REG_CTRL0, 0x41)) return false;

        // 5) Poll Status1 Sat_sensor bit (bit5). Sat_sensor == 0 => PASS per datasheet.
        uint32_t start = millis();
        while (millis() - start < timeout_ms) {
            uint8_t status = 0;
            if (!readRegister(REG_STATUS1, &status)) return false;

            // ST_Fail (bit3) indicates some failure for I3C parity previously; but for self-test we check Sat_sensor
            if ((status & STAT1_SAT_SENSOR) == 0) {
                // Sat_sensor == 0 => PASS
                return true;
            }
            delay(1);
        }

        // Timeout — treat as fail
        return false;
    }

    // ----- Getters / conversions -----

    // Return signed counts (centered about null), using 20-bit assumption
    int32_t signedX() const { return int32_t((int64_t)rawX - NULL_VALUE_20BIT); }
    int32_t signedY() const { return int32_t((int64_t)rawY - NULL_VALUE_20BIT); }
    int32_t signedZ() const { return int32_t((int64_t)rawZ - NULL_VALUE_20BIT); }

    // Return magnetic field in Gauss (float) using 20-bit scaling
    float getFieldGaussX() const { return ((float) signedX()) / COUNTS_PER_G_20BIT; }
    float getFieldGaussY() const { return ((float) signedY()) / COUNTS_PER_G_20BIT; }
    float getFieldGaussZ() const { return ((float) signedZ()) / COUNTS_PER_G_20BIT; }

    // Returns xy angle in range -180, +180
    float getAzimuthDegrees() const {
        float fx = (float) signedX();
        float fy = (float) signedY();
        float ang = atan2f(fy, fx); // radians
        float deg = ang * 180.0f / M_PI;
        return deg;
    }

    // Magnitude (Euclidean) in raw counts and optionally in Gauss
    float getMagnitudeCounts() const {
        float sx = (float) signedX();
        float sy = (float) signedY();
        float sz = (float) signedZ();
        return sqrtf(sx*sx + sy*sy + sz*sz);
    }
    float getMagnitudeGauss() const {
        return getMagnitudeCounts() / COUNTS_PER_G_20BIT;
    }

    // Horizontal magnitude XY plane
    float getHorizontalMagnitudeCounts() const {
        float sx = (float) signedX();
        float sy = (float) signedY();
        return sqrtf(sx*sx + sy*sy);
    }
    float getHorizontalMagnitudeGauss() const {
        return getHorizontalMagnitudeCounts() / COUNTS_PER_G_20BIT;
    }

    bool isContinuousEnabled() const { return _continuous_enabled; }

private:
    TwoWire &_wire;
    bool _continuous_enabled = false;

    // Wait for Meas_m_done bit in STATUS1
    bool waitForMeasurementDone(uint32_t timeout_ms) {
        uint32_t start = millis();
        while (millis() - start < timeout_ms) {
            uint8_t status = 0;
            if (!readRegister(REG_STATUS1, &status)) return false;
            if (status & STAT1_MEAS_M_DONE) return true;
            delay(1);
        }
        return false;
    }

    // Unpack 20-bit unsigned XYZ values from a 9-byte buffer as returned by RDLONG pattern:
    // buf[0] = Xout0 (X[19:12])
    // buf[1] = Xout1 (X[11:4])
    // buf[2] = Yout0 (Y[19:12])
    // buf[3] = Yout1 (Y[11:4])
    // buf[4] = Zout0 (Z[19:12])
    // buf[5] = Zout1 (Z[11:4])
    // buf[6] = Xout2 (X[3:0] << 4) | Yout2 (Y[3:0])
    // buf[7] = Zout2 (Z[3:0] << 4) and maybe next bits depending on ordering
    // buf[8] = Temperature (per example layout)
    void unpackRawXYZFromBuffer(const uint8_t *buf, size_t len) {
        if (len < 9) return;

        // X
        uint32_t x_hi = ((uint32_t)buf[0] << 12);
        uint32_t x_mid = ((uint32_t)buf[1] << 4);
        uint32_t x_lo = ((uint32_t)buf[6] >> 4) & 0x0F;
        rawX = x_hi | x_mid | x_lo;

        // Y
        uint32_t y_hi = ((uint32_t)buf[2] << 12);
        uint32_t y_mid = ((uint32_t)buf[3] << 4);
        uint32_t y_lo = ((uint32_t)buf[6] & 0x0F);
        rawY = y_hi | y_mid | y_lo;

        // Z
        uint32_t z_hi = ((uint32_t)buf[4] << 12);
        uint32_t z_mid = ((uint32_t)buf[5] << 4);
        uint32_t z_lo = ((uint32_t)buf[7] >> 4) & 0x0F;
        rawZ = z_hi | z_mid | z_lo;
    }

    // I2C helpers
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

template <size_t N>
class RollingAverage {
public:
  RollingAverage() : index(0), count(0), sum(0.0f) {
    for (size_t i = 0; i < N; ++i) values[i] = 0.0f;
  }

  void add(float val) {
    if (count < N) {
      sum += val;
      values[index] = val;
      ++count;
    } else {
      sum -= values[index];
      sum += val;
      values[index] = val;
    }
    index = (index + 1) % N;
  }

  float avg() const {
    return (count > 0) ? (sum / count) : 0.0f;
  }

  void reset() {
    index = 0;
    count = 0;
    sum = 0.0f;
    for (size_t i = 0; i < N; ++i) values[i] = 0.0f;
  }

private:
  float values[N];
  size_t index;
  size_t count;
  float sum;
};

#define SDA_PIN 5
#define SCL_PIN 6
#define EMAG_PIN 8

MMC5633NJL mag;

void setup() {
  pinMode(EMAG_PIN, OUTPUT);
  digitalWrite(EMAG_PIN, LOW);
  Serial.begin(250000);
  while(!Serial) {delay(10);};
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mag.begin()) {
    Serial.println("MMC5633NJL init failed");
    while (1) delay(1000);
  }
  Serial.println("MMC5633NJL ready");

  if (!mag.runSelfTest()) {
    Serial.println("SELF TEST FAILED");
  } else {
    Serial.println("Self Test Passed");
  }
  
  delay(100);

  // if (!mag.enableContinuous(50)) {
  //   Serial.println("Failed to enable continuous mode");
  // } else {
  //   Serial.println("Continuous mode 50 Hz enabled");
  // }
}

void loop() {
  uint16_t pwm_clock = 200;
  uint8_t pwm_res = 8;
  uint16_t pwm_setpoint = 255;
  uint16_t time_on_ms = 25;
  uint16_t time_off_ms = 4000;

  uint32_t tnow_ms = 0;
  uint32_t lasttime_ms = 0;
  bool coils_on = false;

  ledcAttachChannel(EMAG_PIN, pwm_clock, pwm_res, 1);
  ledcWrite(EMAG_PIN, 0);

  RollingAverage<200> avgX;
  RollingAverage<200> avgY;
  RollingAverage<200> avgZ;

  while(true) {
    mag.readMeasurement();
    if (!coils_on)
    {
      avgX.add(mag.getFieldGaussX());
      avgY.add(mag.getFieldGaussY());
      avgZ.add(mag.getFieldGaussZ());
    }

    // Serial.print("RawX: "); Serial.print(mag.rawX);
    // Serial.print(" RawY: "); Serial.print(mag.rawY);
    // Serial.print(" RawZ: "); Serial.print(mag.rawZ);
    // Serial.print("GaussX: ");
    // Serial.print(mag.getFieldGaussX(), 4);
    // Serial.print("\tGaussY: ");
    // Serial.print(mag.getFieldGaussY(), 4);
    // Serial.print("\tGaussZ: ");
    // Serial.println(mag.getFieldGaussZ(), 4);
    // Serial.print("\tAzimuth: "); Serial.print(mag.getAzimuthDegrees(), 2);
    // Serial.print(" deg\tMagnitude: "); Serial.print(mag.getMagnitudeGauss(), 4);
    // Serial.println(" G");Serial.print("GaussX: ");
    Serial.print(mag.getFieldGaussX() - avgX.avg(), 4);
    Serial.print(",");
    Serial.print(mag.getFieldGaussY() - avgY.avg(), 4);
    Serial.print(",");
    Serial.print(mag.getFieldGaussZ() - avgZ.avg(), 4);
    Serial.println();
    
    tnow_ms = millis();
    if (coils_on && ((tnow_ms - lasttime_ms) > time_on_ms))
    {
      ledcWrite(EMAG_PIN, 0);
      coils_on = false;
      lasttime_ms = tnow_ms;
    }
    if (!coils_on && ((tnow_ms - lasttime_ms) > time_off_ms))
    {
      ledcWrite(EMAG_PIN, pwm_setpoint);
      coils_on = true;
      lasttime_ms = tnow_ms;
    }
    
    // delay(1);
  }
}
