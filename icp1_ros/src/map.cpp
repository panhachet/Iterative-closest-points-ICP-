#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>

class MapNode : public rclcpp::Node {
public:
    MapNode()
    : Node("map_node"), map_width(200), map_height(200), map_resolution(0.1), origin_x(-10.0), origin_y(-10.0) {
        rclcpp::QoS qos_profile(rclcpp::KeepLast(100));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        laser_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_profile, std::bind(&MapNode::laser_callback, this, std::placeholders::_1));
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapNode::odom_callback, this, std::placeholders::_1));
        map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        map_msg.info.resolution = map_resolution;
        map_msg.info.width = map_width;
        map_msg.info.height = map_height;
        map_msg.info.origin.position.x = origin_x;
        map_msg.info.origin.position.y = origin_y;
        map_msg.data.resize(map_width * map_height, -1); 
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x = msg->pose.pose.position.x;
        robot_y = msg->pose.pose.position.y;
        auto q = msg->pose.pose.orientation;
        robot_theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max) {
                continue; 
            }

        
            float angle = msg->angle_min + i * msg->angle_increment;

            float x_robot = range * cos(angle);
            float y_robot = range * sin(angle);

            float x_map = robot_x + x_robot * cos(robot_theta) - y_robot * sin(robot_theta);
            float y_map = robot_y + x_robot * sin(robot_theta) + y_robot * cos(robot_theta);

       
            int mx = static_cast<int>((x_map - origin_x) / map_resolution);
            int my = static_cast<int>((y_map - origin_y) / map_resolution);

            int robot_mx = static_cast<int>((robot_x - origin_x) / map_resolution);
            int robot_my = static_cast<int>((robot_y - origin_y) / map_resolution);
            bresenham(robot_mx, robot_my, mx, my);

            if (mx >= 0 && mx < map_width && my >= 0 && my < map_height) {
                int index = my * map_width + mx; 
                map_msg.data[index] = 100; 
            }
        }

        map_msg.header.stamp = this->get_clock()->now();
        map_msg.header.frame_id = "map";
        map_publisher->publish(map_msg);
    }

    void bresenham(int x0, int y0, int x1, int y1) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;

        while (true) {
            if (x0 >= 0 && x0 < map_width && y0 >= 0 && y0 < map_height) {
                int index = y0 * map_width + x0;
                if (map_msg.data[index] != 100) { 
                    map_msg.data[index] = 0;
                }
            }

            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;

    nav_msgs::msg::OccupancyGrid map_msg;
    int map_width;
    int map_height;
    double map_resolution;
    double origin_x;
    double origin_y;

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_theta = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapNode>());
    rclcpp::shutdown();
    return 0;
}
