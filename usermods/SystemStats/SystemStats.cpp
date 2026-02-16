#include "wled.h"
#include <WiFi.h>

#ifndef USERMOD_ID_SYSTEM_STATS
#define USERMOD_ID_SYSTEM_STATS  10003
#endif

class SystemStatsUsermod : public Usermod
{

private:
  bool isEnabled = true;
  unsigned long lastPublish = 0;
  unsigned long publishInterval = 10000;  // 10s default
  static constexpr unsigned long MIN_PUBLISH_INTERVAL = 5000;

  static const char _name[];
  static const char _enabled[];
  static const char _publishInterval[];

public:
  void setup() {}

  void loop()
  {
    if (!isEnabled || millis() - lastPublish <= publishInterval) return;
    lastPublish = millis();

    #ifndef WLED_DISABLE_MQTT
    if (!WLED_MQTT_CONNECTED) return;

    // Build JSON payload
    char payload[256];
    snprintf_P(payload, sizeof(payload),
      PSTR("{\"rssi\":%d,\"heap\":%u,\"uptime\":%lu,\"fps\":%u,\"bri\":%u,\"power_mA\":%u,\"wifi_ch\":%d,\"freeheap_min\":%u}"),
      WiFi.RSSI(),
      ESP.getFreeHeap(),
      millis() / 1000,
      strip.getFps(),
      bri,
      BusManager::currentMilliamps(),
      WiFi.channel(),
      ESP.getMinFreeHeap()
    );

    char topic[64];
    snprintf_P(topic, sizeof(topic), PSTR("%s/sys"), mqttDeviceTopic);
    mqtt->publish(topic, 0, false, payload);
    #endif
  }

  void addToJsonInfo(JsonObject& root) {
    if (!isEnabled) return;

    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");

    JsonArray rssiArr = user.createNestedArray(F("WiFi RSSI"));
    rssiArr.add(WiFi.RSSI());
    rssiArr.add(F("dBm"));

    JsonArray heapArr = user.createNestedArray(F("Free Heap"));
    heapArr.add(ESP.getFreeHeap() / 1024);
    heapArr.add(F("kB"));

    JsonArray fpsArr = user.createNestedArray(F("FPS"));
    fpsArr.add(strip.getFps());
  }

  void addToConfig(JsonObject &root)
  {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_enabled)] = isEnabled;
    top[FPSTR(_publishInterval)] = publishInterval / 1000;  // show as seconds
  }

  void appendConfigData()
  {
    oappend(F("addInfo('System Stats:Publish Interval', 1, 's');"));
  }

  bool readFromConfig(JsonObject &root)
  {
    JsonObject top = root[FPSTR(_name)];
    bool configComplete = !top.isNull();
    configComplete &= getJsonValue(top[FPSTR(_enabled)], isEnabled);
    unsigned long intervalSec = publishInterval / 1000;
    configComplete &= getJsonValue(top[FPSTR(_publishInterval)], intervalSec);
    publishInterval = max(intervalSec * 1000, MIN_PUBLISH_INTERVAL);
    return configComplete;
  }

  uint16_t getId()
  {
    return USERMOD_ID_SYSTEM_STATS;
  }
};

const char SystemStatsUsermod::_name[] PROGMEM = "System Stats";
const char SystemStatsUsermod::_enabled[] PROGMEM = "Enabled";
const char SystemStatsUsermod::_publishInterval[] PROGMEM = "Publish Interval";

static SystemStatsUsermod system_stats;
REGISTER_USERMOD(system_stats);
