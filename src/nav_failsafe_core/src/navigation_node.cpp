#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp" // 📡 Offboard Heartbeat Header
#include "px4_msgs/msg/trajectory_setpoint.hpp"    // 🛸 Actuation Command Header
#include <vector>
#include <queue>
#include <cmath>
#include <map>
#include <algorithm>

struct Point3D {
    int x, y, z;
    bool operator<(const Point3D& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
    bool operator==(const Point3D& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct AStarNode {
    Point3D point;
    Point3D parent;
    double g_cost;
    double h_cost;
    double f_cost() const { return g_cost + h_cost; }
    bool operator>(const AStarNode& other) const { return f_cost() > other.f_cost(); }
};

class NavigationFailsafeNode : public rclcpp::Node {
public:
    NavigationFailsafeNode() : Node("nav_failsafe_node") {
        RCLCPP_INFO(this->get_logger(), "🚀 Fully Actuated 3D A* Offboard Node Initialized!");

        // 🛜 Subscribers Setup
        battery_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
            "/fmu/out/battery_status", rclcpp::SensorDataQoS(),
            std::bind(&NavigationFailsafeNode::battery_callback, this, std::placeholders::_1));

        position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            std::bind(&NavigationFailsafeNode::position_callback, this, std::placeholders::_1));

        // 📡 Publishers Setup (PX4 Actuation)
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        // 10Hz Supervisor & Actuation Loop
        supervisor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&NavigationFailsafeNode::supervisor_loop, this));

        start_pos = {0, 0, 0}; // Home Base
        current_drone_pos = {0, 0, 0};
    }

private:
    void battery_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
        battery_level = msg->remaining * 100.0;
    }

    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        current_drone_pos.x = std::round(msg->x);
        current_drone_pos.y = std::round(msg->y);
        current_drone_pos.z = std::round(-msg->z); // NED to NEU conversion
    }

    bool is_obstacle(Point3D p) {
        if (p.x == 2 && (p.y >= 0 && p.y <= 3) && (p.z >= 0 && p.z <= 2)) {
            return true;
        }
        return false;
    }

    double get_heuristic(Point3D p1, Point3D p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
    }

    void run_3d_a_star(Point3D start, Point3D goal) {
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
        std::map<Point3D, Point3D> travel_history;
        std::map<Point3D, double> best_g_cost;

        open_list.push({start, start, 0.0, get_heuristic(start, goal)});
        best_g_cost[start] = 0.0;

        calculated_path.clear();

        while (!open_list.empty()) {
            AStarNode current = open_list.top();
            open_list.pop();

            // 🎯 GOAL CHECK
            if (current.point == goal) {
                travel_history[current.point] = current.parent; // 👈 YEH VALI LINE ADD KARNI HAI!
                trace_final_path(travel_history, start, goal);
                return;
            }

            travel_history[current.point] = current.parent;

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;

                        Point3D neighbor = {current.point.x + dx, current.point.y + dy, current.point.z + dz};
                        if (is_obstacle(neighbor)) continue;

                        double new_g_cost = current.g_cost + get_heuristic(current.point, neighbor);
                        if (best_g_cost.find(neighbor) == best_g_cost.end() || new_g_cost < best_g_cost[neighbor]) {
                            best_g_cost[neighbor] = new_g_cost;
                            open_list.push({neighbor, current.point, new_g_cost, get_heuristic(neighbor, goal)});
                        }
                    }
                }
            }
        }
    }

    void trace_final_path(std::map<Point3D, Point3D>& history, Point3D start, Point3D goal) {
        Point3D curr = goal;
        while (!(curr == start)) {
            calculated_path.push_back(curr);
            curr = history[curr];
        }
        calculated_path.push_back(start);
        std::reverse(calculated_path.begin(), calculated_path.end());

        RCLCPP_INFO(this->get_logger(), "📍 A* Waypoint Queue Loaded! Total Steps: %zu", calculated_path.size());
    }

    // 📡 Offboard Mode Heartbeat Publish Karne Ka Function
    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true; // Hum drone ko position command de rahe hain
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_control_mode_pub_->publish(msg);
    }

    // 🛸 Target Waypoint par Drone ko Fly karwane ka Function
    void publish_trajectory_setpoint(Point3D target) {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position[0] = target.x;
        msg.position[1] = target.y;
        msg.position[2] = -target.z; // NEU grid se wapas PX4 ke standard negative NED Z mein convert kiya
        msg.yaw = -3.14; // Face home
        trajectory_setpoint_pub_->publish(msg);
    }

    // ⏱️ 10Hz Supervisor & Execution Loop
    void supervisor_loop() {
        // 1. Live telemetry logs print karein jab tak failsafe normal hai
        if (!failsafe_triggered) {
            RCLCPP_INFO(this->get_logger(), "📊 Status -> Batt: %.1f%% | Pos: [%d, %d, %d]", 
                        battery_level, current_drone_pos.x, current_drone_pos.y, current_drone_pos.z);
        }

        // 2. Battery Low Trigger Check
        if (battery_level <= 20.0 && !failsafe_triggered) {
            failsafe_triggered = true;
            path_following_active = true;
            current_wp_idx = 0;
            RCLCPP_WARN(this->get_logger(), "⚠️ BINGO FUEL DETECTED! Initiating Emergency Return-to-Home...");
            run_3d_a_star(current_drone_pos, start_pos);
        }

        // 3. Offboard Command Execution Engine
        if (path_following_active) {
            publish_offboard_control_mode(); // Heartbeat bhejna compulsory hai

            if (current_wp_idx < calculated_path.size()) {
                Point3D target_wp = calculated_path[current_wp_idx];
                double distance = get_heuristic(current_drone_pos, target_wp);

                // Agar target waypoint ke paas (0.8m) pahunch gaye, toh agle par badhein
                if (distance < 0.8) {
                    RCLCPP_INFO(this->get_logger(), "🎯 Reached Waypoint %zu! Moving to next point...", current_wp_idx);
                    current_wp_idx++;
                } else {
                    // Live coordinate setpoint publish karein taaki drone physical move kare
                    publish_trajectory_setpoint(target_wp);
                    RCLCPP_INFO(this->get_logger(), "🛸 Flying to WP [%d, %d, %d] | Distance Left: %.2fm", 
                                target_wp.x, target_wp.y, target_wp.z, distance);
                }
            } else {
                // Saare waypoints khatam = Drone ghar pahunch gaya!
                RCLCPP_INFO(this->get_logger(), "🎉 DRONE SAFELY RETURNED TO HOME BASE! Landing Sequence Engaged.");
                path_following_active = false;
            }
        }
    }

    // ROS2 Comms Components
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::TimerBase::SharedPtr supervisor_timer_;
    
    // Logic Variables
    Point3D start_pos;
    Point3D current_drone_pos;
    std::vector<Point3D> calculated_path;
    size_t current_wp_idx = 0;
    
    float battery_level = 100.0;
    bool failsafe_triggered = false;
    bool path_following_active = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationFailsafeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}