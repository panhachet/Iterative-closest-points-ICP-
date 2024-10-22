#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <vector>
#include <cmath>

class MapPublisherNode : public rclcpp::Node
{
public:
  MapPublisherNode()
    : Node("map_publisher"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
  {

    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&MapPublisherNode::scan_callback, this, std::placeholders::_1));

    // publish ocupancy grid
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    //map initialization
    map_.header.frame_id = "map";
    map_.info.resolution = 0.1; 
    map_.info.width = 100;       
    map_.info.height = 100;      
    map_.info.origin.position.x = -5.0; 
    map_.info.origin.position.y = -5.0;
    map_.data.resize(map_.info.width * map_.info.height, -1); 
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    //obtainthe transformation of the LIDAR frame to the map
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform("map", "base_scan", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Couln't transfrom  base_scan to map: %s", ex.what());
      return;
    }

    // intial position of the robot
    int map_center_x = map_.info.width / 2;
    int map_center_y = map_.info.height / 2;
    // Convert each lidar range to x y coordenates and mark the celd as ocupied
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float range = msg->ranges[i];

      // ignore out of range values
      if (range < msg->range_min || range > msg->range_max) {
        continue;
      }

      // calculate the angle for the range measurement
      float angle = msg->angle_min + i * msg->angle_increment;

      // convert to cartesian coordenates
      float x = range * cos(angle);
      float y = range * sin(angle);

      geometry_msgs::msg::PointStamped scan_point, map_point;
      scan_point.header.frame_id = "base_scan";
      scan_point.point.x = x;
      scan_point.point.y = y;
      scan_point.point.z = 0.0;

      // transform the point from base_scan to the map using ICP
      try {
        tf2::doTransform(scan_point, map_point, transform_stamped);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Couldn't transform the point: %s", ex.what());
        continue;
      }

      // convert the coordenates x y of the map into a grid
      int map_x = static_cast<int>((map_point.point.x / map_.info.resolution) + map_center_x);
      int map_y = static_cast<int>((map_point.point.y / map_.info.resolution) + map_center_y);

      // verify the indices are inside the map
      if (map_x >= 0 && map_x < static_cast<int>(map_.info.width) && 
          map_y >= 0 && map_y < static_cast<int>(map_.info.height)) 
      {

        if (map_.data[map_y * map_.info.width + map_x] == -1) {
          map_.data[map_y * map_.info.width + map_x] = 100;  
        }
      }
    }


    map_.header.stamp = this->now();


    map_publisher_->publish(map_);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  nav_msgs::msg::OccupancyGrid map_;  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
