#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "icp.hpp"
#include <vector>
#include <cmath>

double round_to_two_decimal_places(double value)
{
    return std::round(value * 100.0) / 100.0;
}

void round_store_Q(vector<vector<double>>& store_Q) {
    for (auto& point : store_Q) {
        point[0] = round_to_two_decimal_places(point[0]);  // Round x
        point[1] = round_to_two_decimal_places(point[1]);  // Round y
    }
}


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
    rclcpp::QoS qos_profile(rclcpp::KeepLast(100));
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos_profile, std::bind(&ICPTransformNode::scan_callback, this, std::placeholders::_1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", qos_profile, std::bind(&ICPTransformNode::imu_callback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&ICPTransformNode::odom_callback, this, std::placeholders::_1)
    );

    scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan1",10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  int flag = 0;
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    X[0] = msg->pose.pose.position.x;
    X[1] = msg->pose.pose.position.y;
    }
    
void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w); 
    tf2::Matrix3x3 mat(q);
    double roll, pitch;
    mat.getRPY(roll, pitch, current_imu_yaw_);
    X[2] = current_imu_yaw_;
    flag = 1;
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
      float x_ = range * cos(angle);
      float y_ = range * sin(angle);
      data_x.push_back(x_);
      data_y.push_back(y_);
    }

    if (lidar_data_prev_.empty()) {
      lidar_data_prev_ = icp_.combine_data(data_x, data_y);
      store_data = lidar_data_prev_;
    } else {
      std::vector<std::vector<double>> current_data = icp_.combine_data(data_x, data_y);
      store_data.insert(store_data.end(), current_data.begin(), current_data.end());
      cout << store_data.size() << endl;
      publish_scan1(msg, store_data);
      lidar_data_prev_ = current_data;
    }
 
  }

void publish_scan1(const sensor_msgs::msg::LaserScan::SharedPtr& laser_msg,
                   const std::vector<std::vector<double>>& points)
{
    sensor_msgs::msg::LaserScan scan_msg;

    scan_msg.header.stamp = laser_msg->header.stamp;
    scan_msg.header.frame_id = laser_msg->header.frame_id;

    scan_msg.angle_min = laser_msg->angle_min;
    scan_msg.angle_max = laser_msg->angle_max;
    scan_msg.angle_increment = laser_msg->angle_increment;
    scan_msg.range_min = laser_msg->range_min;
    scan_msg.range_max = laser_msg->range_max;
    scan_msg.ranges.resize(static_cast<size_t>((laser_msg->angle_max - laser_msg->angle_min) / laser_msg->angle_increment + 1), std::numeric_limits<float>::infinity());

    for (const auto& point : points)
    {
        float angle = atan2(point[1], point[0]); 
        size_t index = static_cast<size_t>((angle - laser_msg->angle_min) / laser_msg->angle_increment);
        
        if (index < scan_msg.ranges.size())
        {
            float range = sqrt(point[0] * point[0] + point[1] * point[1]);
            scan_msg.ranges[index] = std::min(scan_msg.ranges[index], range);  
        }
    }
    
    scan_publisher_->publish(scan_msg);
}


  void publish_transform_periodically()
  {
    last_transform_ = X;
    if (!last_transform_.empty()) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "odom";  // El marco global
      t.child_frame_id = "base_link";  // El marco del sensor LIDAR (base_scan)

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
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<std::vector<double>> lidar_data_prev_;
  std::vector<std::vector<double>> store_data;
  std::vector<double> last_transform_;
  std::vector<double> X = {0.0, 0.0, 0.0};
  icp icp_;
  double last_imu_yaw_;
  double current_imu_yaw_;
  bool fin = false;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICPTransformNode>());
  rclcpp::shutdown();
  return 0;
}