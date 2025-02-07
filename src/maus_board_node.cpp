#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp> // For temperature, voltage, current, etc.
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/msg/vector3.hpp> // For angular velocity and linear acceleration

#include "maus_board.h"

using namespace std::chrono_literals;

class MausBoardNode : public rclcpp::Node
{
public:
  MausBoardNode() : Node("maus_board_driver"), maus_board_(std::make_unique<MausBoard>(nullptr,nullptr))
  {
    colours_.resize(3);
    RCLCPP_INFO(this->get_logger(), "Starting up MausBoardNode..."); // Debug message

    // Declare parameters
    this->declare_parameter<int>("min_servo_value", 1200);
    this->declare_parameter<int>("max_servo_value", 1800);
    this->declare_parameter<int>("servo_centre", 1500);
    this->declare_parameter<float>("servo_scale", 500.0f);

    if (!maus_board_->startReading())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to start reading from MausBoard");
      rclcpp::shutdown(); // Or handle the error appropriately
    }

    // Initialize subscriber for steering angle and LED[0]
    steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "steering_angle", 10,
        std::bind(&MausBoardNode::steering_callback, this, std::placeholders::_1));
    // colours_.
    led0_sub = this->create_subscription<std_msgs::msg::UInt32>(
        "led0", 10,
        std::bind(&MausBoardNode::led0_callback, this, std::placeholders::_1));
  }

  ~MausBoardNode()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down MausBoardNode..."); // Debug message
    maus_board_->stopReading();
    RCLCPP_INFO(this->get_logger(), "MausBoard stopped."); // Debug message
  }

private:
  void steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    steering_angle_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received steering angle: %f", steering_angle_);
    // Convert steering angle to servo command (example mapping)
    uint16_t steering_command = mapSteeringAngleToServo(steering_angle_);

    // Send servo command (assuming throttle is constant or controlled elsewhere)
    maus_board_->sendSetServos(steering_command, 1500); // Example: Neutral throttle
  }

  void led0_callback(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    colours_.at(0) = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received led0 colour : %u", colours_.at(0));
    maus_board_->sentSetRGB(colours_);
  }

  uint16_t mapSteeringAngleToServo(float steering_angle)
  {
    int min_servo_value_ = this->get_parameter("min_servo_value").as_int();
    int max_servo_value_ = this->get_parameter("max_servo_value").as_int();
    int servo_centre_ = this->get_parameter("servo_centre").as_int();
    float servo_scale_ = this->get_parameter("servo_scale").as_double();

    int mapped_value = servo_centre_ + steering_angle * servo_scale_;
    mapped_value = std::max(min_servo_value_, std::min(max_servo_value_, mapped_value));

    return static_cast<uint16_t>(std::max(mapped_value,0)); //ensure positive
  }

  std::unique_ptr<MausBoard> maus_board_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr led0_sub;
  float steering_angle_; // Store the received steering angle
  float throttle_;       // Store the received throttle
  std::vector<uint32_t> colours_;
  };

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MausBoardNode>());
  rclcpp::shutdown();
  return 0;
}