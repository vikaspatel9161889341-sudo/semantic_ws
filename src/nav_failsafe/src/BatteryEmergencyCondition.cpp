#include <string>
#include <memory>
#include <cmath>
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

class BatteryEmergencyCondition : public BT::ConditionNode
{
public:
  BatteryEmergencyCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config), current_battery_(1.0), pose_received_(false)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Pure PX4 Native Battery Topic
    battery_sub_ = node_->create_subscription<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status", 10,
      [this](const px4_msgs::msg::BatteryStatus::SharedPtr msg) { 
        current_battery_ = msg->remaining; 
      });

    // Pure PX4 Local Position Topic
    pose_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", 10,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        if (!pose_received_) {
          home_x_ = msg->x;
          home_y_ = msg->y;
          home_z_ = msg->z;
          pose_received_ = true;
        }
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;
      });
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    if (!pose_received_) {
      return BT::NodeStatus::SUCCESS;
    }

    // 3D Euclidean Distance Math
    double distance = std::sqrt(
      std::pow(current_x_ - home_x_, 2) +
      std::pow(current_y_ - home_y_, 2) +
      std::pow(current_z_ - home_z_, 2)
    );

    double discharge_rate = 0.005; 
    double safety_buffer = 0.15;   
    double e_required = (distance * discharge_rate) + safety_buffer;

    if (current_battery_ <= e_required) {
      RCLCPP_WARN(node_->get_logger(), "🚨 [BT INTERCEPT] Bingo Fuel Reached! Preempting Mission for RTL...");
      return BT::NodeStatus::FAILURE; 
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pose_sub_;

  double current_battery_;
  bool pose_received_;
  float current_x_, current_y_, current_z_;
  float home_x_, home_y_, home_z_;
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) { factory.registerNodeType<BatteryEmergencyCondition>("BatteryEmergencyCondition"); }
