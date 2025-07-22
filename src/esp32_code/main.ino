#include <ESP32Servo.h>
#include <ros2_arduino_interface.h>

// Servo pins
#define BASE_PIN 13
#define SHOULDER_PIN 12
#define ELBOW_PIN 14
#define GRIPPER_PIN 27

// Analog feedback pins
#define BASE_FB_PIN 34
#define SHOULDER_FB_PIN 35
#define ELBOW_FB_PIN 32
#define GRIPPER_FB_PIN 33

// Servo objects
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;

// PID parameters
float kp = 2.0, ki = 0.1, kd = 0.05;
float setPoints[4] = {0.0, 0.0, 0.0, 0.0}; 
float currentAngles[4] = {0.0, 0.0, 0.0, 0.0};
float errors[4] = {0.0, 0.0, 0.0, 0.0};
float integral[4] = {0.0, 0.0, 0.0, 0.0};
float lastError[4] = {0.0, 0.0, 0.0, 0.0};

// ROS2 node and topics
ros2::Node node;
ros2::Publisher<std_msgs::Float32MultiArray> feedback_pub("/arm_feedback", 10);
ros2::Subscriber<std_msgs::Float32MultiArray> joint_sub("/arm_joint_angles", &jointCallback);

void setup() {
  
  Serial.begin(115200);
  ros2::init(&Serial);

  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  gripperServo.attach(GRIPPER_PIN);

  node = ros2::Node("arm_controller");
  node.create_publisher(feedback_pub);
  node.create_subscriber(joint_sub);

  baseServo.write(90);
  shoulderServo.write(90);
  elbowServo.write(90);
  gripperServo.write(90);
}

void jointCallback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data.size() >= 4) {
    setPoints[0] = msg.data[0]; 
    setPoints[1] = msg.data[1]; 
    setPoints[2] = msg.data[2]; 
    setPoints[3] = msg.data[3]; 
  }
}

float readJointAngle(int pin) {
  int raw = analogRead(pin);
  return map(raw, 0, 4095, 0, 180);
}

float computePID(int jointIndex, float dt) {
  float error = setPoints[jointIndex] - currentAngles[jointIndex];
  integral[jointIndex] += error * dt;
  float derivative = (error - lastError[jointIndex]) / dt;
  lastError[jointIndex] = error;
  errors[jointIndex] = error;
  return kp * error + ki * integral[jointIndex] + kd * derivative;
}

void loop() {
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  currentAngles[0] = readJointAngle(BASE_FB_PIN);
  currentAngles[1] = readJointAngle(SHOULDER_FB_PIN);
  currentAngles[2] = readJointAngle(ELBOW_FB_PIN);
  currentAngles[3] = readJointAngle(GRIPPER_FB_PIN);

  float pidOutputs[4];
  for (int i = 0; i < 4; i++) {
    pidOutputs[i] = computePID(i, dt);
  }

  baseServo.write(constrain(90 + pidOutputs[0], 0, 180));
  shoulderServo.write(constrain(90 + pidOutputs[1], 0, 180));
  elbowServo.write(constrain(90 + pidOutputs[2], 0, 180));
  gripperServo.write(constrain(90 + pidOutputs[3], 0, 180));

  std_msgs::Float32MultiArray feedback_msg;
  feedback_msg.data = currentAngles;
  feedback_pub.publish(feedback_msg);
  ros2::spin(&node);

  delay(10);
}