#include "rclcpp/rclcpp.hpp"
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
        RCLCPP_INFO(this->get_logger(), "🚀 3D A* Node with Failsafe Supervisor Initialized!");

        // 10Hz Supervisor Timer (Har 100ms yaani 0.1s mein chalega)
        supervisor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&NavigationFailsafeNode::supervisor_loop, this));

        start_pos = {0, 0, 0};
        goal_pos = {4, 4, 2};

        // Pehla normal path plan karein
        run_3d_a_star(start_pos, goal_pos);
    }

private:
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

        RCLCPP_INFO(this->get_logger(), "📍 Generated Path:");
        for (const auto& wp : final_path) {
            RCLCPP_INFO(this->get_logger(), "   -> Waypoint: [%d, %d, %d]", wp.x, wp.y, wp.z);
        }
    }

    // ⏱️ 10Hz Supervisor Loop
    void supervisor_loop() {
        if (failsafe_triggered) return;

        // Drone ki battery har 0.1 second mein 0.5% kam ho rahi hai (Simulation)
        battery_level -= 0.5;

        // Agar battery 20% (Bingo Fuel threshold) par aa jaye, toh failsafe lagao!
        if (battery_level <= 20.0) {
            failsafe_triggered = true;
            RCLCPP_WARN(this->get_logger(), "⚠️ BINGO FUEL DETECTED! Battery at %.1f%%", battery_level);
            RCLCPP_WARN(this->get_logger(), "🚨 Failsafe Triggered! Rerouting instantly to Home [0,0,0]...");
            
            // Dummy current location se ghar (0,0,0) ka naya rasta nikalein
            Point3D current_drone_pos = {1, 3, 1}; 
            run_3d_a_star(current_drone_pos, start_pos);
        }
    }

    rclcpp::TimerBase::SharedPtr supervisor_timer_;
    Point3D start_pos;
    Point3D goal_pos;
    
    // Failsafe State Variables
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