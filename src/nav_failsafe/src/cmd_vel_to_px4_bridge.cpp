#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

using namespace std::chrono_literals;

class CmdVelToPx4Bridge : public rclcpp::Node
{
public:
  CmdVelToPx4Bridge() : Node("cmd_vel_to_px4_bridge_node")
  {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&CmdVelToPx4Bridge::cmdVelCallback, this, std::placeholders::_1));

    px4_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);

    px4_offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);

    // 🎯 FIX: 10Hz Continuous Timer to keep PX4 Offboard Mode Alive always!
    timer_ = this->create_wall_timer(100ms, std::bind(&CmdVelToPx4Bridge::timerCallback, this));

    // Initialize safe empty velocities
    vx_ = 0.0; vy_ = 0.0; vz_ = 0.0; yaw_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "🔌 Smart Velocity Bridge Active with 10Hz Heartbeat!");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Capture latest velocities from ROS 2 Network
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    vz_ = msg->linear.z;
    yaw_ = msg->angular.z;
  }

  void timerCallback()
  {
    uint64_t current_time = this->get_clock()->now().nanoseconds() / 1000;

    // Publish Offboard Control Mode Heartbeat constantly
    auto offboard_msg = px4_msgs::msg::OffboardControlMode();
    offboard_msg.timestamp = current_time;
    offboard_msg.position = false;
    offboard_msg.velocity = true;
    offboard_msg.acceleration = false;
    offboard_msg.attitude = false;
    offboard_msg.body_rate = false;
    px4_offboard_mode_pub_->publish(offboard_msg);

    // Publish current Trajectory Setpoint
    auto setpoint_msg = px4_msgs::msg::TrajectorySetpoint();
    setpoint_msg.timestamp = current_time;
    setpoint_msg.position = {static_cast<float>(NAN), static_cast<float>(NAN), static_cast<float>(NAN)};
    setpoint_msg.acceleration = {static_cast<float>(NAN), static_cast<float>(NAN), static_cast<float>(NAN)};

    // Map to PX4 NED Frame
    setpoint_msg.velocity[0] = vx_;  
    setpoint_msg.velocity[1] = -vy_; 
    setpoint_msg.velocity[2] = -vz_; 
    setpoint_msg.yaw = yaw_;          

    px4_setpoint_pub_->publish(setpoint_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr px4_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr px4_offboard_mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float vx_, vy_, vz_, yaw_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelToPx4Bridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
