#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
      icp_(std::vector<std::vector<double>>{}, std::vector<std::vector<double>>{}, {0.0, 0.0, 0.0}, 500, 1e-3),
      last_imu_yaw_(0.0),
      current_imu_yaw_(0.0),
      timer_(this->create_wall_timer(100ms, std::bind(&ICPTransformNode::publish_transform_periodically, this)))
  {
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ICPTransformNode::scan_callback, this, std::placeholders::_1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&ICPTransformNode::imu_callback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&ICPTransformNode::odom_callback, this, std::placeholders::_1)
    );

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    X[0] = msg->pose.pose.position.x;
    X[1] = msg->pose.pose.position.y;
}


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

    if (lidar_data_prev_.empty()) {
      lidar_data_prev_ = icp_.combine_data(data_x, data_y);
    } else {
      std::vector<std::vector<double>> current_data = icp_.combine_data(data_x, data_y);
      icp_.set_P(current_data);
      icp_.set_Q(lidar_data_prev_);
      icp_.set_X(X);
      

      auto [aligned_data, transform] = icp_.icp_function();
      lidar_data_prev_.insert(lidar_data_prev_.end(), aligned_data.begin(), aligned_data.end());
      last_transform_ = transform;
      


      double yaw_diff = current_imu_yaw_ - last_imu_yaw_;
      last_transform_[2] += yaw_diff;

      lidar_data_prev_ = aligned_data;

      last_imu_yaw_ = current_imu_yaw_;
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);

    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    current_imu_yaw_ = std::atan2(siny_cosp, cosy_cosp);  
    X = {0.0, 0.0, current_imu_yaw_};
  }

  void publish_transform_periodically()
  {
    if (!last_transform_.empty()) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "map";  // El marco global
      t.child_frame_id = "base_scan";  // El marco del sensor LIDAR (base_scan)

      // Transformación obtenida del ICP para alinear el LIDAR con el marco global
      t.transform.translation.x = last_transform_[0];
      t.transform.translation.y = last_transform_[1];
      t.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, last_transform_[2]);  // Aplicar la rotación en el eje Z
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      // Publicar la transformación para que otros nodos puedan usarla
      tf_broadcaster_->sendTransform(t);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<std::vector<double>> lidar_data_prev_;
  std::vector<double> last_transform_;
  std::vector<double> X = {0.0, 0.0, 0.0};
  icp icp_;
  double last_imu_yaw_;
  double current_imu_yaw_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICPTransformNode>());
  rclcpp::shutdown();
  return 0;
}
