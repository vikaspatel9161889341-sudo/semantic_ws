#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp" // 🛸 PX4 Local Position Header
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
        RCLCPP_INFO(this->get_logger(), "🚀 3D A* Node with Full PX4 Telemetry Initialized!");

        // 🛜 1. Battery Subscriber
        battery_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
            "/fmu/out/battery_status", rclcpp::SensorDataQoS(),
            std::bind(&NavigationFailsafeNode::battery_callback, this, std::placeholders::_1));

        // 🛜 2. Local Position Subscriber
        position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            std::bind(&NavigationFailsafeNode::position_callback, this, std::placeholders::_1));

        // 10Hz Supervisor Timer
        supervisor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&NavigationFailsafeNode::supervisor_loop, this));

        start_pos = {0, 0, 0}; // Home Base
        current_drone_pos = {0, 0, 0}; // Default start position
    }

private:
    // 🔋 Battery Callback
    void battery_callback(const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
        battery_level = msg->remaining * 100.0;
    }

    // 🛸 Position Callback (Live 3D Coordinates Tracking)
    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        // Float values ko round karke hamare integer Point3D grid mein convert kar rahe hain
        current_drone_pos.x = std::round(msg->x);
        current_drone_pos.y = std::round(msg->y);
        current_drone_pos.z = std::round(-msg->z); // NED to NEU (Negative Z ko Positive Height banaya)
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

        while (!open_list.empty()) {
            AStarNode current = open_list.top();
            open_list.pop();

            if (current.point == goal) {
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
        std::vector<Point3D> final_path;
        Point3D curr = goal;

        while (!(curr == start)) {
            final_path.push_back(curr);
            curr = history[curr];
        }
        final_path.push_back(start);
        std::reverse(final_path.begin(), final_path.end());

        RCLCPP_INFO(this->get_logger(), "🚨 EMERGENCY RETURN PATH GENERATED:");
        for (const auto& wp : final_path) {
            RCLCPP_INFO(this->get_logger(), "   -> [X: %d, Y: %d, Z: %d]", wp.x, wp.y, wp.z);
        }
    }

    // ⏱️ 10Hz Supervisor Loop
    void supervisor_loop() {
        if (failsafe_triggered) return;

        // Log logs for clarity
        RCLCPP_INFO(this->get_logger(), "📊 Status -> Batt: %.1f%% | Pos: [%d, %d, %d]", 
                    battery_level, current_drone_pos.x, current_drone_pos.y, current_drone_pos.z);

        if (battery_level <= 20.0) {
            failsafe_triggered = true;
            RCLCPP_WARN(this->get_logger(), "⚠️ BINGO FUEL DETECTED! Battery: %.1f%%", battery_level);
            RCLCPP_WARN(this->get_logger(), "🚨 Initiating A* Return-to-Home from live position!");
            
            // Asli current position se ghar (start_pos) ka rasta plan karein
            run_3d_a_star(current_drone_pos, start_pos);
        }
    }

    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
    rclcpp::TimerBase::SharedPtr supervisor_timer_;
    
    Point3D start_pos;
    Point3D current_drone_pos;
    
    float battery_level = 100.0;
    bool failsafe_triggered = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationFailsafeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}