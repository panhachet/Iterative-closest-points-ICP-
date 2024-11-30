#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

class map_node : public rclcpp::Node
{
    public:
    map_node()
    :  Node("map_node"), map_width(200), map_height(200), map_resolution(0.1), origin_x(-10.0), origin_y(-10.0)
    {
        rclcpp::QoS qos_profile(rclcpp::KeepLast(100));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        new_scan_sub = this->create_subscription<visualization_msgs::msg::Marker>(
            "new_scan", 10,
            std::bind(&map_node::scan_callback, this, std::placeholders::_1)
        );

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
            std::bind(&map_node::odom_callback, this, std::placeholders::_1)
        );

        map_msg.info.resolution = map_resolution;
        map_msg.info.width = map_width;
        map_msg.info.height = map_height;
        map_msg.info.origin.position.x = origin_x;
        map_msg.info.origin.position.y = origin_y;
        map_msg.data.resize(map_width * map_height, -1);
    }
    private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_pose[0]  = msg->pose.pose.position.x;
        robot_pose[1] = msg->pose.pose.position.y;
        auto q = msg->pose.pose.orientation;
        robot_pose[2]  = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y  + q.z*q.z));
    }

    void scan_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
    {
        for(const auto &point : msg->points)
        {
            int mx = static_cast<int>((point.x - origin_x)/map_resolution);
            int my = static_cast<int>((point.y - origin_y)/map_resolution);
            cout << mx << "\t" << my << endl;
            int rmx = static_cast<int>((robot_pose[0] - origin_x)/map_resolution);
            int rmy = static_cast<int>((robot_pose[1] - origin_y)/map_resolution);
            bresenham(rmx, rmy, mx, my);

            if (mx >= 0 && mx < map_width && my >= 0 && my < map_height)
            {
                int index = my * map_width + mx;
                map_msg.data[index] = 100;
            }

        }
        map_msg.header.stamp = this->get_clock()->now();
        map_msg.header.frame_id = "map";
        std::cout << "ok" << std::endl;
        map_pub->publish(map_msg);
    }

    void bresenham(int x0, int y0, int x1, int y1)
    {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2; 

        while (true)
        {
            if (x0 >= 0 && x0 < map_width && y0 >= 0 && y0 < map_height)
            {
                int index = y0 * map_width + x0;
                if (map_msg.data[index] != 100)
                {
                    map_msg.data[index] = 0; 
                }
            }
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) {err += dy; x0 += sx;}
            if (e2 <= dx) {err += dx; y0 += sy;}
        }
        
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr new_scan_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    nav_msgs::msg::OccupancyGrid  map_msg;
    int map_width;
    int map_height;
    double map_resolution;
    double origin_x;
    double origin_y;
    std::vector<double> robot_pose = {0.0, 0.0, 0.0};

    std::vector<double> X[3];
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<map_node>());
    rclcpp::shutdown();
    return 0;
}