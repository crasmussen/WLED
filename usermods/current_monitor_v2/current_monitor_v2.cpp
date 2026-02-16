#include "wled.h"

#ifndef USERMOD_ID_CURRENT_MONITOR
#define USERMOD_ID_CURRENT_MONITOR  9999  // Assign a unique ID for the Current Monitor usermod
#endif

class CurrentMonitorUsermod : public Usermod
{

private:
  static constexpr unsigned long minLoopInterval = 1000;  // minimum allowable interval (ms)
  unsigned long loopInterval = 100;
  unsigned long lastTime = 0;
  bool isEnabled = true;
  uint8_t previousPlaylist = 0;         // Stores the playlist that was active before high-temperature activation
  uint8_t previousPreset = 0;           // Stores the preset that was active before high-temperature activation
  uint8_t presetToActivate = 0;         // Preset to activate when temp goes above threshold (0 = disabled)
  float activationThreshold = 30.0f;    // Temperature threshold to trigger high-temperature actions
  float resetMargin = 2.0f;             // Margin below the activation threshold (Prevents frequent toggling when close to threshold)
  bool isAboveThreshold = false;        // Flag to track if the high temperature preset is currently active

  static const int PIN_CURRENT = A0;        // ADC pin for current sensor
  static const int SAMPLE_INTERVAL = 10;    // ~10ms between samples
  static constexpr float TRIP_CURRENT_A = 10.0f; // shut-off threshold in Amps

  static constexpr float ADC_TO_AMPS_SLOPE  = 0.00829253f;   // e.g. 0.01352 A per ADC unit
  static constexpr float ADC_TO_AMPS_OFFSET = -2.87447f;     // e.g. small offset in Amps

  unsigned long lastSample = 0;
  float currentAverage = 0.0f;
  bool tripped = false;

  // Circular buffer for 10-sample moving average
  static const uint8_t BUFFER_SIZE = 10;
  float samples[BUFFER_SIZE] = {0};
  uint8_t idx = 0;
  bool bufferFull = false;

  // MQTT publishing
  unsigned long lastMqttPublish = 0;
  static constexpr unsigned long MQTT_PUBLISH_INTERVAL = 10000;  // 10s

  static const char _name[];
  static const char _enabled[];
  static const char _loopInterval[];
  static const char _activationThreshold[];
  static const char _presetToActivate[];

public:
  void setup()
  {
    pinMode(PIN_CURRENT, INPUT);
  }

  void loop()
  {
    // if usermod is disabled or called during strip updating just exit
    // NOTE: on very long strips strip.isUpdating() may always return true so update accordingly
    if (!isEnabled || millis() - lastTime <= loopInterval)
    {
      return;
    }

    lastTime = millis();

    // Read ADC (0–4095 on ESP32)
    int raw = analogRead(PIN_CURRENT);
    float amps = raw * ADC_TO_AMPS_SLOPE + ADC_TO_AMPS_OFFSET;
    if (amps < 0.0f) amps = 0.0f;  // clamp negative

    samples[idx] = amps;
    idx = (idx + 1) % BUFFER_SIZE;
    if (idx == 0) bufferFull = true;

    float sum = 0.0f;
    uint8_t count = bufferFull ? BUFFER_SIZE : idx;
    for (uint8_t i = 0; i < count; i++) sum += samples[i];
    currentAverage = sum / count;

    // Over-current protection
    if (currentAverage > TRIP_CURRENT_A && !tripped) {
      tripped = true;
      bri = 0;  // turn off LEDs
      colorUpdated(CALL_MODE_DIRECT_CHANGE);
      Serial.printf("CURRENT TRIP! %.2f A > %.0f A\n", currentAverage, TRIP_CURRENT_A);
    } else if (tripped && currentAverage < TRIP_CURRENT_A - 0.5f) {  // 0.5A hysteresis
      tripped = false;
      Serial.println(F("Current safe again"));
    }

    // Publish to MQTT
    #ifndef WLED_DISABLE_MQTT
    if (WLED_MQTT_CONNECTED && millis() - lastMqttPublish > MQTT_PUBLISH_INTERVAL) {
      lastMqttPublish = millis();
      char buf[64];
      snprintf_P(buf, sizeof(buf), PSTR("%s/current"), mqttDeviceTopic);
      mqtt->publish(buf, 0, false, String(currentAverage, 3).c_str());
    }
    #endif

  }

  void addToJsonInfo(JsonObject& root) {
    if (!_enabled) return;

    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");

    // Current + status
    JsonArray currArr = user.createNestedArray(F("Current"));
    currArr.add(roundf(currentAverage * 1000.0f) / 1000.0f);
    currArr.add(tripped ? F("A TRIPPED!") : F("A"));
  }

  void addToJsonState(JsonObject& root) {
    root["current_avg_A"] = roundf(currentAverage * 100.0f) / 100.0f;
    root["current_tripped"] = tripped;
  }

  void addToConfig(JsonObject &root)
  {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_enabled)] = isEnabled;
    top[FPSTR(_loopInterval)] = loopInterval;
    top[FPSTR(_activationThreshold)] = activationThreshold;
    top[FPSTR(_presetToActivate)] = presetToActivate;
  }

    // Append useful info to the usermod settings gui
    void appendConfigData()
    {
    // Display 'ms' next to the 'Loop Interval' setting
    oappend(F("addInfo('Internal Temperature:Loop Interval', 1, 'ms');"));
    // Display '°C' next to the 'Activation Threshold' setting
    oappend(F("addInfo('Internal Temperature:Activation Threshold', 1, '°C');"));
    // Display '0 = Disabled' next to the 'Preset To Activate' setting
    oappend(F("addInfo('Internal Temperature:Preset To Activate', 1, '0 = unused');"));
    }

  bool readFromConfig(JsonObject &root)
  {
    JsonObject top = root[FPSTR(_name)];
    bool configComplete = !top.isNull();
    configComplete &= getJsonValue(top[FPSTR(_enabled)], isEnabled);
    configComplete &= getJsonValue(top[FPSTR(_loopInterval)], loopInterval);
    loopInterval = max(loopInterval, minLoopInterval);    // Makes sure the loop interval isn't too small.
    configComplete &= getJsonValue(top[FPSTR(_presetToActivate)], presetToActivate);
    configComplete &= getJsonValue(top[FPSTR(_activationThreshold)], activationThreshold);
    return configComplete;
  }

  uint16_t getId()
  {
    return USERMOD_ID_CURRENT_MONITOR;
  }
};

const char CurrentMonitorUsermod::_name[] PROGMEM = "Current Monitor";
const char CurrentMonitorUsermod::_enabled[] PROGMEM = "Enabled";
const char CurrentMonitorUsermod::_loopInterval[] PROGMEM = "Loop Interval";
const char CurrentMonitorUsermod::_activationThreshold[] PROGMEM = "Activation Threshold";
const char CurrentMonitorUsermod::_presetToActivate[] PROGMEM = "Preset To Activate";

static CurrentMonitorUsermod current_monitor_v2;
REGISTER_USERMOD(current_monitor_v2);