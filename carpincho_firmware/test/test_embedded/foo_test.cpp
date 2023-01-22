#include <Arduino.h>
#include <foo.h>
#include <unity.h>

void setUp(void) {
  // Set stuff up here.
}

void tearDown(void) {
  // Clean stuff up here.
}

void foo_test_construct_ok(void) {
  carpinchobot::Foo foo;
  TEST_ASSERT_EQUAL(0, foo.get());
}

void foo_test_set_value(void) {
  carpinchobot::Foo foo;
  foo.set(1);
  TEST_ASSERT_EQUAL(1, foo.get());
}

void RUN_ALL_TESTS() {
  UNITY_BEGIN();
  RUN_TEST(foo_test_construct_ok);
  RUN_TEST(foo_test_set_value);
  UNITY_END();
}

void setup() {
  // Wait for 2 secs in case board doesn't support software reset via
  // Serial.DTR/RTS.
  delay(2000);

  RUN_ALL_TESTS();
}

void loop() {
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(500);
}
