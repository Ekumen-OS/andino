#include <PIDController.h>

// HW
#define ENCODER_1_PIN 2
#define ENCODER_2_PIN 3

#define MOTOR_1_ENA_PIN 9
#define MOTOR_1_IN1_PIN 4
#define MOTOR_1_IN2_PIN 5
#define MOTOR_2_ENB_PIN 10
#define MOTOR_2_IN3_PIN 6
#define MOTOR_2_IN4_PIN 7

#define COMPUTE_WINDOW_MS 50
#define ENCODER_TICKS_PER_REV 20

#define KP 250
#define KI 5
#define KD 0

unsigned long last_compute_time = 0;

volatile long encoder_1_ticks = 0;
volatile long encoder_2_ticks = 0;

PIDController pid_motor_1;
PIDController pid_motor_2;

unsigned long encoder_1_ticks_delta_time = 1;
unsigned long last_tick_time_1 = 0;
unsigned long encoder_2_ticks_delta_time = 1;
unsigned long last_tick_time_2 = 0;

void encoder1Callback() {
  encoder_1_ticks++;
  encoder_1_ticks_delta_time = millis() - last_tick_time_1;
  last_tick_time_1 = millis();
}

void encoder2Callback() {
  encoder_2_ticks++;
  encoder_2_ticks_delta_time = millis() - last_tick_time_2;
  last_tick_time_2 = millis();
}

void setup() {
  // Motors setup
  pinMode(MOTOR_1_ENA_PIN, OUTPUT);
  pinMode(MOTOR_1_IN1_PIN, OUTPUT);
  pinMode(MOTOR_1_IN2_PIN, OUTPUT);
  pinMode(MOTOR_2_ENB_PIN, OUTPUT);
  pinMode(MOTOR_2_IN3_PIN, OUTPUT);
  pinMode(MOTOR_2_IN4_PIN, OUTPUT);

  // Encoders setup.
  pinMode(ENCODER_1_PIN, INPUT);
  pinMode(ENCODER_2_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN), encoder1Callback, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), encoder2Callback, RISING);

  // TODO: Support going backwards.
  digitalWrite(MOTOR_1_IN1_PIN, HIGH);
  digitalWrite(MOTOR_1_IN2_PIN, LOW);
  digitalWrite(MOTOR_2_IN3_PIN, HIGH);
  digitalWrite(MOTOR_2_IN4_PIN, LOW);

  digitalWrite(MOTOR_1_ENA_PIN, HIGH);
  digitalWrite(MOTOR_2_ENB_PIN, HIGH);

  // PID setup
  pid_motor_1.begin();
  pid_motor_1.setpoint(38.0);
  pid_motor_1.tune(KP, KI, KD);
  pid_motor_1.limit(150, 255);
  pid_motor_2.begin();
  pid_motor_2.setpoint(28.0);
  pid_motor_2.tune(KP, KI, KD);
  pid_motor_2.limit(150, 255);

  //setMotor1Setpoint(38.0);
  //setMotor2Setpoint(28.0);

  // Serial port for debugging purposes.
  Serial.begin(9600);
}

void loop() {
  unsigned long elapsed_time = millis() - last_compute_time;   
  if (elapsed_time > COMPUTE_WINDOW_MS) {
    double motor_1_ticks_per_second = (double)1000 / encoder_1_ticks_delta_time;
    int control_action_1 = pid_motor_1.compute(motor_1_ticks_per_second);
    setMotor1Speed(control_action_1);

    double motor_2_ticks_per_second = (double)1000 / encoder_2_ticks_delta_time;
    int control_action_2 = pid_motor_2.compute(motor_2_ticks_per_second);
    setMotor2Speed(control_action_2);

    // Debug
    Serial.print(motor_1_ticks_per_second);
    Serial.print(",");
    Serial.print(control_action_1);
    Serial.print(",");
    Serial.print(motor_2_ticks_per_second);
    Serial.print(",");
    Serial.println(control_action_2);

    last_compute_time = millis();
  }
}

void setMotor1Setpoint(double setpoint) {
  pid_motor_1.setpoint(setpoint);
}

void setMotor2Setpoint(double setpoint) {
  pid_motor_2.setpoint(setpoint);
}

void setMotor1Speed(int speed) {
  //analogWrite(MOTOR_1_ENA_PIN, abs(speed));
}

void setMotor2Speed(int speed) {
  //analogWrite(MOTOR_2_ENB_PIN, abs(speed));
}

// Convert number of encoder ticks to angle in radians.
double ticksToAngle(unsigned long ticks) {
  return ticks * (2.0 * M_PI / ENCODER_TICKS_PER_REV);
}
