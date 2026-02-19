#include "wled.h"
#include <Wire.h>

#ifndef USERMOD_ID_TMP411_TEMPERATURE
#define USERMOD_ID_TMP411_TEMPERATURE  10001
#endif

// TMP411 I2C Register Addresses
#define TMP411_REG_LOCAL_TEMP_HIGH   0x00
#define TMP411_REG_LOCAL_TEMP_LOW    0x15
#define TMP411_REG_CONFIG            0x03
#define TMP411_REG_CONVERSION_RATE   0x04
#define TMP411_REG_MANUFACTURER_ID   0xFE
#define TMP411_REG_DEVICE_ID         0xFF

// TMP411 Expected IDs
#define TMP411_MANUFACTURER_ID       0x55  // Texas Instruments
#define TMP411_DEVICE_ID_VALUE       0x12  // TMP411

class TMP411TemperatureUsermod : public Usermod
{

private:
  static constexpr unsigned long MIN_LOOP_INTERVAL = 1000;  // minimum 1s between reads
  unsigned long loopInterval = 2000;   // default 2s polling
  unsigned long lastTime = 0;
  bool isEnabled = true;
  bool sensorFound = false;
  bool initDone = false;

  uint8_t i2cAddress = 0x4C;  // TMP411A default address (DADDFR = "A" address variant)
  float temperature = -127.0f;

  // High-temperature preset activation
  uint8_t presetToActivate = 0;
  float activationThreshold = 70.0f;
  float resetMargin = 5.0f;
  bool isAboveThreshold = false;
  uint8_t previousPreset = 0;
  uint8_t previousPlaylist = 0;

  // MQTT publishing
  unsigned long lastMqttPublish = 0;
  static constexpr unsigned long MQTT_PUBLISH_INTERVAL = 10000;  // 10s

  static const char _name[];
  static const char _enabled[];
  static const char _loopInterval[];
  static const char _i2cAddress[];
  static const char _activationThreshold[];
  static const char _presetToActivate[];

  uint8_t readRegister8(uint8_t reg, bool *ok = nullptr) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {  // repeated start, no stop
      if (ok) *ok = false;
      return 0;
    }
    if (Wire.requestFrom(i2cAddress, (uint8_t)1) == 0 || !Wire.available()) {
      if (ok) *ok = false;
      return 0;
    }
    if (ok) *ok = true;
    return Wire.read();
  }

  bool detectSensor() {
    Wire.beginTransmission(i2cAddress);
    if (Wire.endTransmission() != 0) return false;

    uint8_t mfgId = readRegister8(TMP411_REG_MANUFACTURER_ID);
    if (mfgId != TMP411_MANUFACTURER_ID) {
      Serial.printf("TMP411: unexpected manufacturer ID 0x%02X (expected 0x%02X)\n", mfgId, TMP411_MANUFACTURER_ID);
      return false;
    }
    return true;
  }

  float readLocalTemperature() {
    bool okH = false, okL = false;
    uint8_t highByte = readRegister8(TMP411_REG_LOCAL_TEMP_HIGH, &okH);
    uint8_t lowByte  = readRegister8(TMP411_REG_LOCAL_TEMP_LOW, &okL);

    if (!okH) return -127.0f;

    // Combine into 16-bit value and convert:
    // temp_mC = raw * 125 / 32  (from Linux hwmon driver)
    int16_t raw = ((int16_t)highByte << 8) | lowByte;
    float tempC = (raw * 125.0f) / 32.0f / 1000.0f;

    return tempC;
  }

public:
  void setup()
  {
    // Defer init to loop() so serial is ready and I2C bus is fully up
  }

  void initSensor()
  {
    initDone = true;

    Serial.printf("TMP411: isEnabled=%d, SDA=%d, SCL=%d\n", isEnabled, i2c_sda, i2c_scl);
    if (!isEnabled) return;
    if (i2c_scl < 0 || i2c_sda < 0) {
      Serial.println(F("TMP411: I2C pins not configured"));
      return;
    }

    // I2C bus scan for debugging
    Serial.println(F("TMP411: I2C bus scan..."));
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf("TMP411:   device at 0x%02X\n", addr);
      }
    }

    sensorFound = detectSensor();
    if (sensorFound) {
      // Set conversion rate to 4 Hz (index 0x08) for responsive readings
      Wire.beginTransmission(i2cAddress);
      Wire.write(0x0A);  // conversion rate write register
      Wire.write(0x08);  // 4 conversions/sec
      Wire.endTransmission();
      Serial.println(F("TMP411: sensor detected and configured"));
    } else {
      Serial.printf("TMP411: sensor not found at 0x%02X\n", i2cAddress);
    }
  }

  void loop()
  {
    if (!initDone && millis() > 5000) {
      initSensor();  // run once, 5s after boot so serial is ready
    }
    if (!isEnabled || !sensorFound || millis() - lastTime <= loopInterval) {
      return;
    }
    lastTime = millis();

    temperature = readLocalTemperature();
    temperature = roundf(temperature * 10.0f) / 10.0f;  // round to 0.1°C

    // Publish to MQTT
    #ifndef WLED_DISABLE_MQTT
    if (WLED_MQTT_CONNECTED && millis() - lastMqttPublish > MQTT_PUBLISH_INTERVAL) {
      lastMqttPublish = millis();
      char topic[64];
      char payload[96];
      snprintf_P(topic, sizeof(topic), PSTR("%s/board_temp"), mqttDeviceTopic);
      snprintf_P(payload, sizeof(payload),
        PSTR("{\"t\":%lu,\"temp_c\":%.1f}"),
        (unsigned long)toki.second(), temperature);
      mqtt->publish(topic, 0, false, payload);
    }
    #endif

    // High-temperature preset activation
    if (presetToActivate > 0) {
      if (!isAboveThreshold && temperature >= activationThreshold) {
        isAboveThreshold = true;
        previousPreset = currentPreset;
        previousPlaylist = currentPlaylist;
        applyPreset(presetToActivate);
        Serial.printf("TMP411: %.1f°C >= %.1f°C, activating preset %d\n",
                       temperature, activationThreshold, presetToActivate);
      } else if (isAboveThreshold && temperature < (activationThreshold - resetMargin)) {
        isAboveThreshold = false;
        if (previousPlaylist > 0) {
          applyPreset(previousPlaylist);
        } else if (previousPreset > 0) {
          applyPreset(previousPreset);
        }
        Serial.printf("TMP411: %.1f°C < %.1f°C, restoring previous preset\n",
                       temperature, activationThreshold - resetMargin);
      }
    }
  }

  void addToJsonInfo(JsonObject& root) {
    if (!isEnabled) return;

    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");

    JsonArray tempArr = user.createNestedArray(F("Board Temp"));
    if (sensorFound && temperature > -100.0f) {
      tempArr.add(temperature);
      tempArr.add(F("°C"));
    } else {
      tempArr.add(F("sensor not found"));
    }
  }

  void addToJsonState(JsonObject& root) {
    if (!isEnabled || !sensorFound) return;
    root["board_temp_C"] = roundf(temperature * 10.0f) / 10.0f;
  }

  void addToConfig(JsonObject &root)
  {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_enabled)] = isEnabled;
    top[FPSTR(_loopInterval)] = loopInterval;
    top[FPSTR(_i2cAddress)] = i2cAddress;
    top[FPSTR(_activationThreshold)] = activationThreshold;
    top[FPSTR(_presetToActivate)] = presetToActivate;
  }

  void appendConfigData()
  {
    oappend(F("addInfo('Board Temperature:Loop Interval', 1, 'ms');"));
    oappend(F("addInfo('Board Temperature:I2C Address', 1, '76=0x4C, 77=0x4D, 78=0x4E');"));
    oappend(F("addInfo('Board Temperature:Activation Threshold', 1, '&deg;C');"));
    oappend(F("addInfo('Board Temperature:Preset To Activate', 1, '0 = unused');"));
  }

  bool readFromConfig(JsonObject &root)
  {
    JsonObject top = root[FPSTR(_name)];
    bool configComplete = !top.isNull();
    configComplete &= getJsonValue(top[FPSTR(_enabled)], isEnabled);
    configComplete &= getJsonValue(top[FPSTR(_loopInterval)], loopInterval);
    loopInterval = max(loopInterval, MIN_LOOP_INTERVAL);
    configComplete &= getJsonValue(top[FPSTR(_i2cAddress)], i2cAddress);
    configComplete &= getJsonValue(top[FPSTR(_activationThreshold)], activationThreshold);
    configComplete &= getJsonValue(top[FPSTR(_presetToActivate)], presetToActivate);
    return configComplete;
  }

  uint16_t getId()
  {
    return USERMOD_ID_TMP411_TEMPERATURE;
  }
};

const char TMP411TemperatureUsermod::_name[] PROGMEM = "Board Temperature";
const char TMP411TemperatureUsermod::_enabled[] PROGMEM = "Enabled";
const char TMP411TemperatureUsermod::_loopInterval[] PROGMEM = "Loop Interval";
const char TMP411TemperatureUsermod::_i2cAddress[] PROGMEM = "I2C Address";
const char TMP411TemperatureUsermod::_activationThreshold[] PROGMEM = "Activation Threshold";
const char TMP411TemperatureUsermod::_presetToActivate[] PROGMEM = "Preset To Activate";

static TMP411TemperatureUsermod tmp411_temperature;
REGISTER_USERMOD(tmp411_temperature);
