#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
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
      icp_(std::vector<std::vector<double>>{}, std::vector<std::vector<double>>{}, {0.0, 0.0, 0.0}, 10, 1e-3),
      last_imu_yaw_(0.0),
      current_imu_yaw_(0.0)
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
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan1",10);
  }

private:
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
    // X[2] = current_imu_yaw_;

  }
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    X[0] = msg->pose.pose.position.x;
    X[1] = msg->pose.pose.position.y;
    double roll, pitch;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3(q).getRPY(roll, pitch, X[2]);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = "base_scan"; 
    transformStamped.child_frame_id = "odom";  

    transformStamped.transform.translation.x = X[0];
    transformStamped.transform.translation.y = X[1];
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = qx;
    transformStamped.transform.rotation.y = qy;
    transformStamped.transform.rotation.z = qz;
    transformStamped.transform.rotation.w = qw;

    // Send the transform
    tf_broadcaster_->sendTransform(transformStamped);
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
      diff_x = X[0] - diff_x;
      diff_y = X[1] - diff_y;
      float x1 = cos(X[2]) * x_ - sin(X[2]) * y_;
      float y1 = sin(X[2]) * x_ + cos(X[2]) * y_;

      data_x.push_back(x1);
      data_y.push_back(y1);
    }

    if (lidar_data_prev_.empty()) {
      lidar_data_prev_ = icp_.combine_data(data_x, data_y);
      store_data = lidar_data_prev_;
    } else {
        std::vector<std::vector<double>> current_data = icp_.combine_data(data_x, data_y);
        icp_.set_P(current_data);
        icp_.set_Q(store_data);
        // auto [after_icp, dx, success] = icp_.icp_function();
       std::vector<std::vector<double>> after_icp = current_data;
        //if (success)
        //{
            store_data.insert(store_data.end(), after_icp.begin(), after_icp.end());
            round_store_Q(store_data);
            sort(store_data.begin(), store_data.end());
            store_data.erase(unique(store_data.begin(), store_data.end()), store_data.end());
            cout << store_data.size() << endl;
            
            icp_.set_X(X);
            // publish_scan1(msg, store_data);
            update_map(store_data);
        //}
        
    }
 
  }
  void update_map(const std::vector<std::vector<double>>& updated_points)
    {
        
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->get_clock()->now();
        map_msg.header.frame_id = "map";

        map_msg.info.resolution = 0.05;
        map_msg.info.width = 500;       
        map_msg.info.height = 500;      
        map_msg.info.origin.position.x = -12.5; 
        map_msg.info.origin.position.y = -12.5;
        map_msg.data.resize(map_msg.info.width * map_msg.info.height, -1); 
        for (const auto& point : updated_points)
        {
            int x_idx = static_cast<int>((point[0] - map_msg.info.origin.position.x) / map_msg.info.resolution);
            int y_idx = static_cast<int>((point[1] - map_msg.info.origin.position.y) / map_msg.info.resolution);
            if (x_idx >= 0 && x_idx < map_msg.info.width && y_idx >= 0 && y_idx < map_msg.info.height)
            {
                int index = y_idx * map_msg.info.width + x_idx;
                map_msg.data[index] = 100; 
            }
        }
        map_pub_->publish(map_msg);
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


  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<std::vector<double>> lidar_data_prev_;
  std::vector<std::vector<double>> store_data;
  std::vector<double> last_transform_;
  std::vector<double> X = {0.0, 0.0, 0.0};
  icp icp_;
  double last_imu_yaw_;
  double current_imu_yaw_;
  bool fin = false;
  float diff_x, diff_y;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICPTransformNode>());
  rclcpp::shutdown();
  return 0;
}

