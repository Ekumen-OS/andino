#include <ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

#define LOOP_FREQ 30

// Pinout
#define ENCODER_LEFT_PIN 2
#define ENCODER_RIGHT_PIN 3

#define MOTOR_LEFT_ENA_PIN 9
#define MOTOR_LEFT_IN1_PIN 4
#define MOTOR_LEFT_IN2_PIN 5
#define MOTOR_RIGHT_ENB_PIN 10
#define MOTOR_RIGHT_IN3_PIN 6
#define MOTOR_RIGHT_IN4_PIN 7

// Node handle
ros::NodeHandle nh;
// Wheel control
float left_wheel_control = 0.0, right_wheel_control = 0.0;
void setup_pinout();
void left_wheel_control_cb(const std_msgs::Float32& msg);
void right_wheel_control_cb(const std_msgs::Float32& msg);
void process_wheel_control();
ros::Subscriber<std_msgs::Float32> left_wheel_control_sub("left_wheel_control", &left_wheel_control_cb);
ros::Subscriber<std_msgs::Float32> right_wheel_control_sub("right_wheel_control", &right_wheel_control_cb);
// Joint state
void setup_joint_state();
void process_and_publish_joint_state();
sensor_msgs::JointState joint_state_msg;
ros::Publisher joint_state_pub("joint_state", &joint_state_msg);

/**
 * @brief Callback for left wheel control
 *
 * @param[in] Float32 input message
 */
void left_wheel_control_cb(const std_msgs::Float32& msg) {
  left_wheel_control = msg.data;
}

/**
 * @brief Callback for right wheel control
 *
 * @param[in] Float32 input message
 */
void right_wheel_control_cb(const std_msgs::Float32& msg) {
  right_wheel_control = msg.data;
}

/**
 * @brief Sets the wheel motors based on the control messages received
 */
void process_wheel_control() {
  // TODO: Process incoming data
}

/**
 * @brief Allocates memory for the joint state message
 */
void setup_joint_state() {
  joint_state_msg.name = malloc(sizeof(char*) * 2);
  joint_state_msg.position = (float*)malloc(sizeof(float) * 2);
  joint_state_msg.velocity = (float*)malloc(sizeof(float) * 2);
  joint_state_msg.name_length = 2;
  joint_state_msg.position_length = 2;
  joint_state_msg.velocity_length = 2;
  joint_state_msg.name[0] = malloc(sizeof(char) * 15);
  joint_state_msg.name[1] = malloc(sizeof(char) * 15);
  strcpy(joint_state_msg.name[0], "left_wheel");
  strcpy(joint_state_msg.name[1], "right_wheel");
}

/**
 * @briefSetup I/O pinout
 */
void setup_pinout() {
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);

  pinMode(MOTOR_LEFT_ENA_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_IN1_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_IN2_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_ENB_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4_PIN, OUTPUT);
}

/**
 * @brief After reading the data from the enconders, sets a publish joint states
 */
void process_and_publish_joint_state() {
  joint_state_msg.position[0] = 0.0;
  joint_state_msg.position[1] = 0.0;
  joint_state_msg.velocity[0] = left_wheel_control;
  joint_state_msg.velocity[1] = right_wheel_control;
  joint_state_pub.publish(&joint_state_msg);
}

void setup() {
  nh.initNode();

  setup_pinout();
  setup_joint_state();

  nh.subscribe(left_wheel_control_sub);
  nh.subscribe(right_wheel_control_sub);
  nh.advertise(joint_state_pub);
}

void loop() {
  nh.spinOnce();
  process_wheel_control();
  process_and_publish_joint_state();
  delay(1000 / LOOP_FREQ);
}
