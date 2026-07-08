#include <Arduino.h>

#define LED_PIN 13

bool led_on = false;

void setup() {
  delay(1500);

  Serial.begin(115200);
  
  Serial.println("Hello world!");

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  led_on ? (digitalWrite(LED_PIN, LOW), led_on = false) : (digitalWrite(LED_PIN, HIGH), led_on = true);
  led_on ? Serial.println("ON") : Serial.println("OFF");
  delay(1000);
}