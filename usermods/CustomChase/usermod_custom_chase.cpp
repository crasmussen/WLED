#include "wled.h"

/*
 * Custom Chase Effect Usermod for WLED
 * 
 * This usermod adds a custom chase effect that can be selected
 * from the WLED effects list.
 */

#ifndef USERMOD_ID_CUSTOM_CHASE
#define USERMOD_ID_CUSTOM_CHASE 242
#endif

class CustomChaseUsermod : public Usermod {
  private:
    bool enabled = true;
    
    // Effect mode ID - use very high number to avoid conflicts
    static const uint8_t FX_MODE_CUSTOM_CHASE = 254;
    
  public:
    
    void setup() {
      Serial.println("=== CustomChase Usermod Setup ===");
      Serial.printf("Effect ID: %d\n", FX_MODE_CUSTOM_CHASE);
      Serial.printf("Effect name: %s\n", _data_FX_MODE_CUSTOM_CHASE);
      
      // Register the custom effect
      strip.addEffect(FX_MODE_CUSTOM_CHASE, &customChaseEffect, _data_FX_MODE_CUSTOM_CHASE);
      
      Serial.printf("Total effects registered: %d\n", strip.getModeCount());
      Serial.println("=== CustomChase Setup Complete ===");
    }
    
    void loop() {
      // Usermod loop - can be used for non-effect logic if needed
    }
    
    /*
     * Custom Chase Effect Implementation for RGBW LEDs
     * This is called by WLED's effect engine
     */
    static uint16_t customChaseEffect(void) {
      WS2812FX* strip_ptr = &strip;
      Segment& seg = strip_ptr->getSegment(strip_ptr->getCurrSegmentId());
      
      // Get effect parameters from WLED sliders
      uint16_t delay_ms = 20 + ((255 - seg.speed) * 2);  // Speed slider
      uint8_t intensity = seg.intensity;  // Intensity slider
      uint32_t color = seg.colors[0];  // Primary color (includes white channel)
      
      // Calculate chase length based on intensity
      uint8_t chase_len = 2 + (intensity / 32);  // 2-10 LEDs
      
      // Clear all LEDs in segment
      seg.fill(BLACK);
      
      // Get current position (cycles through segment)
      uint16_t pos = strip_ptr->now / delay_ms;
      pos = pos % seg.virtualLength();
      
      // Draw the chase
      for (uint8_t i = 0; i < chase_len; i++) {
        // Calculate brightness fade for trail effect
        uint8_t brightness = 255 - (i * (255 / chase_len));
        uint32_t faded_color = color_fade_rgbw(color, brightness, seg.hasWhite());
        
        // Set LED with wrap-around
        uint16_t led_pos = (pos + seg.virtualLength() - i) % seg.virtualLength();
        seg.setPixelColor(led_pos, faded_color);
      }
      
      return delay_ms;
    }
    
    /*
     * Helper function to fade an RGBW color by brightness
     * Handles both RGB and RGBW strips automatically
     */
    static uint32_t color_fade_rgbw(uint32_t color, uint8_t brightness, bool hasWhite) {
      uint8_t w = (color >> 24) & 0xFF;  // White channel
      uint8_t r = (color >> 16) & 0xFF;  // Red
      uint8_t g = (color >> 8) & 0xFF;   // Green
      uint8_t b = color & 0xFF;          // Blue
      
      // Apply brightness scaling
      r = (r * brightness) / 255;
      g = (g * brightness) / 255;
      b = (b * brightness) / 255;
      
      if (hasWhite) {
        w = (w * brightness) / 255;
        return (w << 24) | (r << 16) | (g << 8) | b;
      } else {
        return (r << 16) | (g << 8) | b;
      }
    }
    
    /*
     * Effect metadata - name shown in WLED UI
     */
    static const char _data_FX_MODE_CUSTOM_CHASE[];
    
    uint16_t getId() {
      return USERMOD_ID_CUSTOM_CHASE;
    }
};

// Define the static const char array for effect metadata
const char CustomChaseUsermod::_data_FX_MODE_CUSTOM_CHASE[] PROGMEM = "Custom Chase@Speed,Length;;;1";

// Create static instance with lowercase name
static CustomChaseUsermod customChaseUsermod;

// Register the usermod instance (not the class!)
REGISTER_USERMOD(customChaseUsermod);