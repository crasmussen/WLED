#include "wled.h"
#include <WiFiUdp.h>

/*
 * Multi-Output Realtime Usermod for WLED - DEBUG VERSION
 * 
 * Listens on UDP port 9999 for RGBW pixel data.
 * 
 * Packet format:
 * - Header: "WLED" (4 bytes)
 * - Data: RGBW values (4 bytes per LED)
 */

#ifndef USERMOD_ID_MULTIOUT_REALTIME
#define USERMOD_ID_MULTIOUT_REALTIME 243
#endif

class MultiOutputRealtimeUsermod : public Usermod {
  private:
    WiFiUDP udp;
    uint16_t udpPort = 9999;
    static const uint16_t MAX_PACKET_SIZE = 2048;
    uint8_t packetBuffer[MAX_PACKET_SIZE];
    
    bool enabled = true;
    bool udpStarted = false;
    
    // Statistics
    unsigned long packetsReceived = 0;
    unsigned long validPackets = 0;
    unsigned long invalidPackets = 0;
    unsigned long lastFpsCheck = 0;
    uint16_t fps = 0;
    uint16_t fpsCounter = 0;
    unsigned long lastDebugPrint = 0;
    
  public:
    
    void setup() {
      Serial.println(F(""));
      Serial.println(F("========================================"));
      Serial.println(F("  MultiOutput Realtime Usermod v2.0"));
      Serial.println(F("========================================"));
      Serial.println(F("Waiting for WiFi connection..."));
    }
    
    void connected() {
      if (udpStarted) return;
      if (!enabled) return;
      
      Serial.println(F(""));
      Serial.println(F("WiFi connected! Starting UDP listener..."));
      
      // Stop any existing UDP
      udp.stop();
      
      if (udp.begin(udpPort)) {
        udpStarted = true;
        
        Serial.println(F("========================================"));
        Serial.printf("UDP listening on port %d\n", udpPort);
        Serial.printf("Local IP: %s\n", Network.localIP().toString().c_str());
        Serial.printf("Max packet size: %d bytes\n", MAX_PACKET_SIZE);
        Serial.println(F("========================================"));
        Serial.printf("Strip total LEDs: %d\n", strip.getLengthTotal());
        Serial.printf("Number of busses: %d\n", BusManager::getNumBusses());
        
        for (uint8_t b = 0; b < BusManager::getNumBusses(); b++) {
          Bus* bus = BusManager::getBus(b);
          if (bus) {
            Serial.printf("  Bus %d: start=%d, length=%d, type=%d\n", 
                          b, bus->getStart(), bus->getLength(), bus->getType());
          }
        }
        Serial.println(F("========================================"));
        Serial.println(F("Waiting for packets with 'WLED' header..."));
        Serial.println(F("========================================"));
        Serial.println(F(""));
      } else {
        Serial.println(F("ERROR: Failed to start UDP!"));
        enabled = false;
      }
    }
    
    void loop() {
      if (!enabled || !udpStarted) return;
      
      // Periodic status print (every 30 seconds)
      if (millis() - lastDebugPrint > 30000) {
        lastDebugPrint = millis();
        Serial.println(F(""));
        Serial.println(F("--- UDP Status ---"));
        Serial.printf("Port: %d, Listening: %s\n", udpPort, udpStarted ? "YES" : "NO");
        Serial.printf("Total packets seen: %lu\n", packetsReceived);
        Serial.printf("Valid WLED packets: %lu\n", validPackets);
        Serial.printf("Invalid packets: %lu\n", invalidPackets);
        Serial.printf("Current realtime mode: %d\n", realtimeMode);
        Serial.printf("Current FPS: %d\n", fps);
        Serial.println(F("------------------"));
        Serial.println(F(""));
      }
      
      // Check for packets
      int packetSize = udp.parsePacket();
      
      if (packetSize > 0) {
        packetsReceived++;
        handlePacket(packetSize);
      }
      
      // Update FPS counter
      if (millis() - lastFpsCheck >= 1000) {
        fps = fpsCounter;
        fpsCounter = 0;
        lastFpsCheck = millis();
      }
    }
    
    void handlePacket(int packetSize) {
      IPAddress remoteIP = udp.remoteIP();
      uint16_t remotePort = udp.remotePort();
      
      // Read packet
      int len = udp.read(packetBuffer, min(packetSize, (int)MAX_PACKET_SIZE));
      
      // Debug: Print info about every packet for first 20
      if (packetsReceived <= 20) {
        Serial.printf("Packet #%lu from %s:%d, size=%d, header: 0x%02X 0x%02X 0x%02X 0x%02X ('%c%c%c%c')\n",
                      packetsReceived,
                      remoteIP.toString().c_str(),
                      remotePort,
                      len,
                      len > 0 ? packetBuffer[0] : 0,
                      len > 1 ? packetBuffer[1] : 0,
                      len > 2 ? packetBuffer[2] : 0,
                      len > 3 ? packetBuffer[3] : 0,
                      len > 0 && packetBuffer[0] >= 32 ? (char)packetBuffer[0] : '.',
                      len > 1 && packetBuffer[1] >= 32 ? (char)packetBuffer[1] : '.',
                      len > 2 && packetBuffer[2] >= 32 ? (char)packetBuffer[2] : '.',
                      len > 3 && packetBuffer[3] >= 32 ? (char)packetBuffer[3] : '.');
      }
      
      // Verify header "WLED"
      // New format: "WLED" (4 bytes) + start_index (2 bytes, little-endian) + RGBW data
      // Minimum: 4 + 2 + 4 = 10 bytes (header + index + 1 pixel)
      if (len < 10 ||
          packetBuffer[0] != 'W' || 
          packetBuffer[1] != 'L' || 
          packetBuffer[2] != 'E' || 
          packetBuffer[3] != 'D') {
        invalidPackets++;
        return;
      }
      
      // Valid packet!
      validPackets++;
      fpsCounter++;
      
      // Parse start index (little-endian uint16 at bytes 4-5)
      uint16_t startIndex = packetBuffer[4] | (packetBuffer[5] << 8);
      
      // Calculate pixel count (subtract header and index)
      int numPixels = (len - 6) / 4;
      uint16_t totalLeds = strip.getLengthTotal();
      
      // Debug first valid packet extensively
      if (validPackets == 1) {
        Serial.println(F(""));
        Serial.println(F("*** FIRST VALID WLED PACKET! ***"));
        Serial.printf("From: %s:%d\n", remoteIP.toString().c_str(), remotePort);
        Serial.printf("Packet size: %d bytes\n", len);
        Serial.printf("Start index: %d\n", startIndex);
        Serial.printf("Pixel count in packet: %d\n", numPixels);
        Serial.printf("Strip total LEDs: %d\n", totalLeds);
        
        // Print first few pixels
        Serial.println(F("First 5 pixels (RGBW):"));
        for (int i = 0; i < 5 && i < numPixels; i++) {
          int off = 6 + i * 4;  // Skip header (4) + index (2)
          Serial.printf("  [%d]: R=%d G=%d B=%d W=%d\n", startIndex + i,
                        packetBuffer[off], packetBuffer[off+1], 
                        packetBuffer[off+2], packetBuffer[off+3]);
        }
        Serial.println(F("*******************************"));
        Serial.println(F(""));
      }
      
      // Enter realtime mode
      realtimeLock(2500, REALTIME_MODE_GENERIC);
      
      // Set pixels starting at startIndex
      for (int i = 0; i < numPixels; i++) {
        uint16_t ledIndex = startIndex + i;
        if (ledIndex >= totalLeds) break;  // Don't overflow
        
        int offset = 6 + (i * 4);  // Skip header (4) + index (2)
        uint8_t r = packetBuffer[offset + 0];
        uint8_t g = packetBuffer[offset + 1];
        uint8_t b = packetBuffer[offset + 2];
        uint8_t w = packetBuffer[offset + 3];
        
        strip.setPixelColor(ledIndex, RGBW32(r, g, b, w));
      }
      
      // Only show after last chunk (start index 0 or when we've received all LEDs)
      // For simplicity, always show - the ESP32 can handle it
      strip.show();
      
      // Periodic valid packet debug
      if (validPackets % 300 == 0) {
        Serial.printf("Valid packets: %lu, FPS: %d\n", validPackets, fps);
      }
    }
    
    void addToJsonInfo(JsonObject& root) {
      if (!enabled) return;
      
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
      
      JsonArray arr = user.createNestedArray(F("UDP RT"));
      if (realtimeMode == REALTIME_MODE_GENERIC) {
        arr.add(validPackets);
        arr.add(F(" pkts @ "));
        arr.add(fps);
        arr.add(F(" fps"));
      } else {
        arr.add(F("Idle ("));
        arr.add(validPackets);
        arr.add(F(" total)"));
      }
    }
    
    void addToConfig(JsonObject& root) {
      JsonObject top = root.createNestedObject(F("MultiOutput RT"));
      top[F("enabled")] = enabled;
      top[F("port")] = udpPort;
    }
    
    bool readFromConfig(JsonObject& root) {
      JsonObject top = root[F("MultiOutput RT")];
      if (top.isNull()) return false;
      
      getJsonValue(top[F("enabled")], enabled);
      getJsonValue(top[F("port")], udpPort);
      
      return true;
    }
    
    uint16_t getId() {
      return USERMOD_ID_MULTIOUT_REALTIME;
    }
};

static MultiOutputRealtimeUsermod multiOutputRealtimeUsermod;
REGISTER_USERMOD(multiOutputRealtimeUsermod);