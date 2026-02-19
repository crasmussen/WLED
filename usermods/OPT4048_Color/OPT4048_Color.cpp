#include "wled.h"
#include <Wire.h>

#ifndef USERMOD_ID_OPT4048_COLOR
#define USERMOD_ID_OPT4048_COLOR  10002
#endif

// OPT4048 Register Addresses
#define OPT4048_REG_CH0_MSB       0x00
#define OPT4048_REG_CH0_LSB       0x01
#define OPT4048_REG_CH1_MSB       0x02
#define OPT4048_REG_CH1_LSB       0x03
#define OPT4048_REG_CH2_MSB       0x04
#define OPT4048_REG_CH2_LSB       0x05
#define OPT4048_REG_CH3_MSB       0x06
#define OPT4048_REG_CH3_LSB       0x07
#define OPT4048_REG_CONTROL       0x0A
#define OPT4048_REG_INT_CONTROL   0x0B
#define OPT4048_REG_FLAGS         0x0C
#define OPT4048_REG_DEVICE_ID     0x11

#define OPT4048_DEVICE_ID_VALUE   0x0821

// CIE 1931 matrix coefficients (from TI datasheet section 9.2.4)
// Rows: X, Y, Z;  Columns: CH0(R), CH1(G), CH2(B), constant
static const double cieMatrix[3][4] = {
  { 0.000234892992,  -0.0000189652390,  0.0000120811684,  0.0     },
  { 0.0000407467441,  0.000198958202,  -0.0000158848115,  0.00215 },
  { 0.0000928619404, -0.0000169739553,  0.000674021520,   0.0     }
};

class OPT4048ColorUsermod : public Usermod
{

private:
  static constexpr unsigned long MIN_LOOP_INTERVAL = 500;
  unsigned long loopInterval = 2000;
  unsigned long lastTime = 0;
  bool isEnabled = true;
  bool sensorFound = false;
  bool initDone = false;

  uint8_t i2cAddress = 0x44;  // OPT4048 default address

  // Computed values
  float lux = 0.0f;
  float cieX = 0.0f;
  float cieY = 0.0f;
  float cct = 0.0f;

  // Raw ADC codes
  uint32_t adcCh[4] = {0};

  // MQTT publishing
  unsigned long lastMqttPublish = 0;
  static constexpr unsigned long MQTT_PUBLISH_INTERVAL = 10000;  // 10s

  static const char _name[];
  static const char _enabled[];
  static const char _loopInterval[];
  static const char _i2cAddress[];

  bool readRegister16(uint8_t reg, uint16_t *val) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(i2cAddress, (uint8_t)2) < 2) return false;
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    *val = ((uint16_t)msb << 8) | lsb;
    return true;
  }

  bool writeRegister16(uint8_t reg, uint16_t val) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    Wire.write((uint8_t)(val >> 8));
    Wire.write((uint8_t)(val & 0xFF));
    return Wire.endTransmission() == 0;
  }

  bool detectSensor() {
    // Check if device ACKs at this address
    Wire.beginTransmission(i2cAddress);
    uint8_t ack = Wire.endTransmission();
    if (ack != 0) {
      Serial.printf("OPT4048: no ACK at 0x%02X (error %d)\n", i2cAddress, ack);
      return false;
    }

    uint16_t devId = 0;
    if (!readRegister16(OPT4048_REG_DEVICE_ID, &devId)) {
      Serial.println(F("OPT4048: failed to read device ID register"));
      return false;
    }
    Serial.printf("OPT4048: device ID = 0x%04X\n", devId);
    if (devId != OPT4048_DEVICE_ID_VALUE) {
      Serial.printf("OPT4048: unexpected device ID (expected 0x%04X)\n", OPT4048_DEVICE_ID_VALUE);
      return false;
    }
    return true;
  }

  // Read a single channel (2 registers: MSB at regMsb, LSB at regMsb+1)
  uint32_t readChannel(uint8_t regMsb) {
    uint16_t msb = 0, lsb = 0;
    if (!readRegister16(regMsb, &msb)) return 0;
    if (!readRegister16(regMsb + 1, &lsb)) return 0;

    // MSB register: [15:12]=exponent, [11:0]=result_msb
    uint8_t exponent = (msb >> 12) & 0x0F;
    uint16_t resultMsb = msb & 0x0FFF;

    // LSB register: [15:8]=result_lsb, [7:4]=counter, [3:0]=crc
    uint8_t resultLsb = (lsb >> 8) & 0xFF;

    uint32_t mantissa = ((uint32_t)resultMsb << 8) | resultLsb;
    return mantissa << exponent;
  }

  void readAllChannels() {
    adcCh[0] = readChannel(OPT4048_REG_CH0_MSB);
    adcCh[1] = readChannel(OPT4048_REG_CH1_MSB);
    adcCh[2] = readChannel(OPT4048_REG_CH2_MSB);
    adcCh[3] = readChannel(OPT4048_REG_CH3_MSB);
  }

  void computeColorMetrics() {
    double ch0 = (double)adcCh[0];
    double ch1 = (double)adcCh[1];
    double ch2 = (double)adcCh[2];

    double X = ch0 * cieMatrix[0][0] + ch1 * cieMatrix[0][1] + ch2 * cieMatrix[0][2] + cieMatrix[0][3];
    double Y = ch0 * cieMatrix[1][0] + ch1 * cieMatrix[1][1] + ch2 * cieMatrix[1][2] + cieMatrix[1][3];
    double Z = ch0 * cieMatrix[2][0] + ch1 * cieMatrix[2][1] + ch2 * cieMatrix[2][2] + cieMatrix[2][3];

    // Lux from Y (green channel with constant)
    lux = (float)(ch1 * cieMatrix[1][3]);
    if (lux < 0.0f) lux = 0.0f;

    // CIE 1931 xy chromaticity
    double sum = X + Y + Z;
    if (sum > 0.0001) {
      cieX = (float)(X / sum);
      cieY = (float)(Y / sum);
    } else {
      cieX = 0.0f;
      cieY = 0.0f;
    }

    // CCT using McCamy's approximation
    if (cieY > 0.0001f) {
      double n = ((double)cieX - 0.3320) / (0.1858 - (double)cieY);
      cct = (float)(437.0 * n * n * n + 3601.0 * n * n + 6861.0 * n + 5517.0);
      if (cct < 0.0f) cct = 0.0f;
    } else {
      cct = 0.0f;
    }
  }

  void configureSensor() {
    // Control register (0x0A):
    // [13:10] range = 0b1100 (auto-range)
    // [9:6]   conversion_time = 0b1000 (100ms)
    // [5:4]   op_mode = 0b11 (continuous)
    // [3]     latch = 1
    // [2]     int_pol = 0
    // [1:0]   fault_count = 0b00
    uint16_t ctrl = (0x0C << 10) | (0x08 << 6) | (0x03 << 4) | (1 << 3);
    writeRegister16(OPT4048_REG_CONTROL, ctrl);

    // Int control register (0x0B):
    // [0] i2c_burst = 1 (enable burst read for speed)
    writeRegister16(OPT4048_REG_INT_CONTROL, 0x0001);
  }

public:
  void setup()
  {
    // Defer init to loop() so serial is ready and I2C bus is fully up
  }

  void initSensor()
  {
    initDone = true;

    Serial.printf("OPT4048: isEnabled=%d, SDA=%d, SCL=%d\n", isEnabled, i2c_sda, i2c_scl);
    if (!isEnabled) return;
    if (i2c_scl < 0 || i2c_sda < 0) {
      Serial.println(F("OPT4048: I2C pins not configured"));
      return;
    }

    sensorFound = detectSensor();
    if (sensorFound) {
      configureSensor();
      Serial.println(F("OPT4048: sensor detected and configured"));
    } else {
      Serial.printf("OPT4048: sensor not found at 0x%02X\n", i2cAddress);
    }
  }

  void loop()
  {
    if (!initDone && millis() > 5500) {
      initSensor();
    }
    if (!isEnabled || !sensorFound || millis() - lastTime <= loopInterval) {
      return;
    }
    lastTime = millis();

    readAllChannels();
    computeColorMetrics();

    // Publish to MQTT
    #ifndef WLED_DISABLE_MQTT
    if (WLED_MQTT_CONNECTED && millis() - lastMqttPublish > MQTT_PUBLISH_INTERVAL) {
      lastMqttPublish = millis();
      char topic[64];
      char payload[160];
      snprintf_P(topic, sizeof(topic), PSTR("%s/color"), mqttDeviceTopic);
      snprintf_P(payload, sizeof(payload),
        PSTR("{\"t\":%lu,\"lux\":%.1f,\"cct\":%d,\"cie_x\":%.4f,\"cie_y\":%.4f}"),
        (unsigned long)toki.second(), lux, (int)roundf(cct), cieX, cieY);
      mqtt->publish(topic, 0, false, payload);
    }
    #endif
  }

  void addToJsonInfo(JsonObject& root) {
    if (!isEnabled) return;

    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");

    if (sensorFound) {
      JsonArray luxArr = user.createNestedArray(F("Lux"));
      luxArr.add(roundf(lux * 10.0f) / 10.0f);
      luxArr.add(F("lx"));

      JsonArray cctArr = user.createNestedArray(F("Color Temp"));
      cctArr.add((int)roundf(cct));
      cctArr.add(F("K"));

      JsonArray cieArr = user.createNestedArray(F("CIE xy"));
      char buf[20];
      snprintf(buf, sizeof(buf), "%.4f, %.4f", cieX, cieY);
      cieArr.add(buf);
    } else {
      JsonArray arr = user.createNestedArray(F("Color Sensor"));
      arr.add(F("not found"));
    }
  }

  void addToJsonState(JsonObject& root) {
    if (!isEnabled || !sensorFound) return;
    JsonObject color = root.createNestedObject("opt4048");
    color["lux"] = roundf(lux * 10.0f) / 10.0f;
    color["cct"] = (int)roundf(cct);
    color["cie_x"] = roundf(cieX * 10000.0f) / 10000.0f;
    color["cie_y"] = roundf(cieY * 10000.0f) / 10000.0f;
  }

  void addToConfig(JsonObject &root)
  {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_enabled)] = isEnabled;
    top[FPSTR(_loopInterval)] = loopInterval;
    top[FPSTR(_i2cAddress)] = i2cAddress;
  }

  void appendConfigData()
  {
    oappend(F("addInfo('Ambient Color:Loop Interval', 1, 'ms');"));
    oappend(F("addInfo('Ambient Color:I2C Address', 1, '68=0x44, 69=0x45, 70=0x46');"));
  }

  bool readFromConfig(JsonObject &root)
  {
    JsonObject top = root[FPSTR(_name)];
    bool configComplete = !top.isNull();
    configComplete &= getJsonValue(top[FPSTR(_enabled)], isEnabled);
    configComplete &= getJsonValue(top[FPSTR(_loopInterval)], loopInterval);
    loopInterval = max(loopInterval, MIN_LOOP_INTERVAL);
    configComplete &= getJsonValue(top[FPSTR(_i2cAddress)], i2cAddress);
    return configComplete;
  }

  uint16_t getId()
  {
    return USERMOD_ID_OPT4048_COLOR;
  }
};

const char OPT4048ColorUsermod::_name[] PROGMEM = "Ambient Color";
const char OPT4048ColorUsermod::_enabled[] PROGMEM = "Enabled";
const char OPT4048ColorUsermod::_loopInterval[] PROGMEM = "Loop Interval";
const char OPT4048ColorUsermod::_i2cAddress[] PROGMEM = "I2C Address";

static OPT4048ColorUsermod opt4048_color;
REGISTER_USERMOD(opt4048_color);
