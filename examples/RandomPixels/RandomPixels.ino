#include <LED_Array_4x5.h>

// Global variables
byte led_pins[] = {0, 1, 2, 3, 4};
int x;
int y;
int state;

void setup() {
  
  // Initialize and clear display
  Plex.init(led_pins);
  Plex.clear();
  Plex.display();

  // Seed our random number generator
  randomSeed(analogRead(A0));
}

void loop() {
  
  // Choose a random X coordinate
  x = random(0, ROW_SIZE);
  
  // Choose a random Y coordinate
  y = random(0, COL_SIZE);
  
  // Flip a coin for the state of the LED
  state = random(0, 2);
  
  // Write to the LED display and wait before doing it again
  Plex.pixel(x, y, state);
  Plex.display();
  
  delay(10);
}
