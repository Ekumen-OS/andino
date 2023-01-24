#include <Arduino.h>
#include <foo.h>

#include "hw.h"

void blink(int times, int delay_ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delay_ms);
    digitalWrite(LED_PIN, LOW);
    delay(delay_ms);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Program running...");
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  carpinchobot::Foo foo;

  foo.set(3);
  Serial.print("Blinks: ");
  Serial.println(foo.get());
  blink(foo.get(), 200);
  delay(2000);

  foo.set(2);
  Serial.print("Blinks: ");
  Serial.println(foo.get());
  blink(foo.get(), 200);
  delay(2000);
}
