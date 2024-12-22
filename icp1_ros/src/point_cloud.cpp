#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LaserToPointCloudNode : public rclcpp::Node {
public:
    LaserToPointCloudNode()
        : Node("laser_to_pointcloud"),
          tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
          tf_listener_(tf_buffer_) {
        
        // Subscribe to the LaserScan topic
        rclcpp::QoS qos_profile(rclcpp::KeepLast(100));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",  // Replace with your scan topic name
            qos_profile,
            std::bind(&LaserToPointCloudNode::scanCallback, this, std::placeholders::_1)
        );

        // Publisher for the PointCloud2
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        try {
            // Convert LaserScan to PointCloud2
            sensor_msgs::msg::PointCloud2 cloud;
            std::cout  << scan_msg->ranges.size() << std::endl;
            projector_.projectLaser(*scan_msg, cloud);
            point_cloud_pub_->publish(cloud);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error converting LaserScan to PointCloud: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserToPointCloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
