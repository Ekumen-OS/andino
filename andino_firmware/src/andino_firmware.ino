/*
 * Arduino IDE / Arduino-core entry points.
 *
 * This sketch file provides the standard `setup()` and `loop()` symbols that
 * the Arduino core (including ESP32) expects, and simply forwards them to the
 * existing andino::App implementation.
 *
 * NOTE: Do not put any application logic here; keep everything in App to
 * ensure the serial protocol and higher-level behavior remain unchanged.
 */

#include "app.h"

void setup() { andino::App::setup(); }

void loop() { andino::App::loop(); }
