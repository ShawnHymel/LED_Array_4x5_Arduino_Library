#include <LED_Array_4x5.h>

byte led_pins[] = {0, 1, 2, 3, 4};

void setup() {
  
  // Initialize and clear display
  Plex.init(led_pins);
  Plex.clear();
  Plex.display();
}

void loop() {
  Plex.pixel(2, 3);
  Plex.display();

  delay(30);
}
