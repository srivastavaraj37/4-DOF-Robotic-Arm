#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <chrono>
#include <string>

class ArmControlNode : public rclcpp::Node {
public:
  ArmControlNode() : Node("arm_control_node"), last_time_(now()) {
    // Initialize PID parameters
    kp_ = {2.0, 2.0, 2.0, 2.0};
    ki_ = {0.1, 0.1, 0.1, 0.1};
    kd_ = {0.05, 0.05, 0.05, 0.05};
    set_points_ = {0.0, 0.0, 0.0, 0.0};
    current_angles_ = {0.0, 0.0, 0.0, 0.0};
    errors_ = {0.0, 0.0, 0.0, 0.0};
    integral_ = {0.0, 0.0, 0.0, 0.0};
    last_error_ = {0.0, 0.0, 0.0, 0.0};

    // Publishers and subscribers
    joint_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/arm_joint_angles", 10);
    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    feedback_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/arm_feedback", 10, std::bind(&ArmControlNode::feedbackCallback, this, std::placeholders::_1));

    // Timer for control loop (100Hz)
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&ArmControlNode::controlLoop, this));

    // Log initialization
    RCLCPP_INFO(this->get_logger(), "Arm control node initialized with PID: Kp=[%.2f, %.2f, %.2f, %.2f], Ki=[%.2f, %.2f, %.2f, %.2f], Kd=[%.2f, %.2f, %.2f, %.2f]",
                kp_[0], kp_[1], kp_[2], kp_[3], ki_[0], ki_[1], ki_[2], ki_[3], kd_[0], kd_[1], kd_[2], kd_[3]);
  }

private:
  void feedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 4) {
      current_angles_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Received feedback: Base=%.2f, Shoulder=%.2f, Elbow=%.2f, Gripper=%.2f",
                  current_angles_[0], current_angles_[1], current_angles_[2], current_angles_[3]);
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid feedback data size: %zu", msg->data.size());
    }
  }

  void controlLoop() {
    auto current_time = now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    set_points_ = {90.0, 45.0, 45.0, 0.0}; // Base, Shoulder, Elbow, Gripper

    // Compute PID for each joint
    std::vector<float> outputs(4);
    for (size_t i = 0; i < 4; ++i) {
      errors_[i] = set_points_[i] - current_angles_[i];
      integral_[i] += errors_[i] * dt;
      float derivative = (errors_[i] - last_error_[i]) / dt;
      outputs[i] = kp_[i] * errors_[i] + ki_[i] * integral_[i] + kd_[i] * derivative;
      last_error_[i] = errors_[i];

      // Log PID details
      RCLCPP_INFO(this->get_logger(), "Joint %zu: Setpoint=%.2f, Current=%.2f, Error=%.2f, Integral=%.2f, Derivative=%.2f, Output=%.2f",
                  i, set_points_[i], current_angles_[i], errors_[i], integral_[i], derivative, outputs[i]);
    }

    // Publish joint angles to ESP32
    auto joint_msg = std_msgs::msg::Float32MultiArray();
    joint_msg.data = outputs;
    joint_pub_->publish(joint_msg);
    RCLCPP_INFO(this->get_logger(), "Published joint angles: [%.2f, %.2f, %.2f, %.2f]",
                outputs[0], outputs[1], outputs[2], outputs[3]);

    // Publish joint states for RViz
    auto state_msg = sensor_msgs::msg::JointState();
    state_msg.header.stamp = now();
    state_msg.name = {"base_joint", "shoulder_joint", "elbow_joint", "gripper_joint"};
    state_msg.position = current_angles_;
    state_pub_->publish(state_msg);
  }

  // PID parameters and state
  std::vector<float> kp_, ki_, kd_;
  std::vector<float> set_points_, current_angles_, errors_, integral_, last_error_;
  rclcpp::Time last_time_;

  // ROS2 interfaces
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr feedback_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}