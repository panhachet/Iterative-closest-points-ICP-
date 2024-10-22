#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "icp.hpp"  
#include <vector>
#include <cmath>

class ICPTransformNode : public rclcpp::Node
{
public:
  ICPTransformNode()
    : Node("icp_transform_node"),
      icp_(std::vector<std::vector<double>>{}, std::vector<std::vector<double>>{}, {0.0, 0.0, 0.0}, 10, 1e-3),
      timer_(this->create_wall_timer(100ms, std::bind(&ICPTransformNode::publish_transform_periodically, this)))
  {
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ICPTransformNode::scan_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<double> data_x, data_y;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float range = msg->ranges[i];
      if (range < msg->range_min || range > msg->range_max) {
        continue;
      }
      float angle = msg->angle_min + i * msg->angle_increment;
      float x = range * cos(angle);
      float y = range * sin(angle);
      data_x.push_back(x);
      data_y.push_back(y);
    }

    // initiate data if we don't have any
    if (lidar_data_prev_.empty()) {
      lidar_data_prev_ = icp_.combine_data(data_x, data_y);
    } else {
      // DO ICP to allign previous data
      std::vector<std::vector<double>> current_data = icp_.combine_data(data_x, data_y);
      icp_.set_P(current_data);  // update current point cloud
      icp_.set_Q(lidar_data_prev_);  // use points for ICP

      auto [aligned_data, transform] = icp_.icp_function();  // Tranformation from ICP


      last_transform_ = transform;

      lidar_data_prev_ = aligned_data; 
    }
  }

  void publish_transform_periodically()
  {
    if (!last_transform_.empty()) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "map";   // Frame global
      t.child_frame_id = "base_scan";  // LIDAR FRAME


      // Assign the transormation obtained from the ICP
      t.transform.translation.x = last_transform_[0];  // Trranformation in x
      t.transform.translation.y = last_transform_[1];  // Transformation in y
      t.transform.translation.z = 0.0;  

      // Asssign rotation in Z
      tf2::Quaternion q;
      q.setRPY(0, 0, last_transform_[2]);//  Rotation in the z axis
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      // publish transform
      tf_broadcaster_->sendTransform(t);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<std::vector<double>> lidar_data_prev_;  
  std::vector<double> last_transform_; 
  icp icp_; 
  rclcpp::TimerBase::SharedPtr timer_;  // Publish the transform each time
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICPTransformNode>());
  rclcpp::shutdown();
  return 0;
}
