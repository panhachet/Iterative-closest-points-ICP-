#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include <vector>

using namespace std;

class LaserScanMarkerPublisher : public rclcpp::Node
{
public:
    LaserScanMarkerPublisher() : Node("laser_scan_marker_publisher")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(100));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&LaserScanMarkerPublisher::scan_callback, this, std::placeholders::_1));

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/new_scan", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int downsample_factor = 10; 
        //obj_x.clear();
        //obj_y.clear();

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = msg->header.frame_id; 
        marker.header.stamp = this->now();
        marker.ns = "laser_scan_points";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.05; 
        marker.scale.y = 0.05; 

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        for (size_t i = 0; i < msg->ranges.size(); i += downsample_factor)
        {
            float range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max)
            {
                continue;
            }

            float angle = msg->angle_min + i * msg->angle_increment;
            float x = range * cos(angle);
            float y = range * sin(angle);

            float obx = x * cos(robot_pose[2]) - y * sin(robot_pose[2]);
            float oby = x * sin(robot_pose[2]) + y * cos(robot_pose[2]);

            obj_x.push_back(obx);
            obj_y.push_back(oby);

            for (size_t i = 0; i < obj_x.size(); ++i)
            {
                geometry_msgs::msg::Point p;
                p.x = obj_x[i];
                p.y = obj_y[i];
                p.z = 0.0;
                marker.points.push_back(p);
            }
          
        }

   
        marker_publisher_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    vector<double> obj_x;
    vector<double> obj_y;
    vector<double> obj_xs;
    vector<double> obj_ys;
    vector<double> robot_pose = {0.0, 0.0, 0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanMarkerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


