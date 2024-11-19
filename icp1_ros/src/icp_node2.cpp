#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class ICPMappingNode : public rclcpp::Node {
public:
    ICPMappingNode() : Node("icp_mapping_with_odometry") {
        rclcpp::QoS qos_profile(rclcpp::KeepLast(100));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", qos_profile, std::bind(&ICPMappingNode::scanCallback, this, std::placeholders::_1));
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&ICPMappingNode::odometryCallback, this, std::placeholders::_1));

        // Publisher
        map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 10);

        RCLCPP_INFO(this->get_logger(), "ICP Mapping Node Initialized.");
    }

private:
    // ROS2 entities
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;

    // PCL Point Clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr old_scan_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_{new pcl::PointCloud<pcl::PointXYZ>()};

    // Odometry Transform
    tf2::Transform last_odometry_transform_{tf2::Transform::getIdentity()};

    // Callback: LaserScan
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_scan(new pcl::PointCloud<pcl::PointXYZ>());
        convertLaserScanToPointCloud(scan_msg, new_scan);

        // Skip ICP if no old scan exists
        if (old_scan_->empty()) {
            *old_scan_ = *new_scan;  // Save the first scan
            *map_ = *new_scan;       // Initialize the map
            publishMap();
            return;
        }

        // Apply odometry as the initial guess for ICP
        Eigen::Matrix4f odom_guess = tf2TransformToEigen(last_odometry_transform_);

        // ICP Alignment
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(new_scan);
        icp.setInputTarget(old_scan_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_scan(new pcl::PointCloud<pcl::PointXYZ>());
        icp.align(*aligned_scan, odom_guess);

        if (icp.hasConverged()) {
            // Transform and merge aligned scan into the map
            pcl::transformPointCloud(*new_scan, *aligned_scan, icp.getFinalTransformation());
            *map_ += *aligned_scan;

            // Update the old scan with the aligned scan
            *old_scan_ = *aligned_scan;
            publishMap();
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        }
    }

    // Callback: Odometry
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        geometry_msgs::msg::Pose pose = odom_msg->pose.pose;
        tf2::fromMsg(pose, last_odometry_transform_);
    }

    // Utility: Convert LaserScan to PointCloud
    void convertLaserScanToPointCloud(
        const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            if (std::isfinite(scan_msg->ranges[i])) {
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                pcl::PointXYZ point;
                point.x = scan_msg->ranges[i] * std::cos(angle);
                point.y = scan_msg->ranges[i] * std::sin(angle);
                point.z = 0.0;
                cloud->push_back(point);
            }
        }
    }

    // Utility: Transform tf2::Transform to Eigen::Matrix4f
    Eigen::Matrix4f tf2TransformToEigen(const tf2::Transform &tf) {
        Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
        auto origin = tf.getOrigin();
        matrix(0, 3) = origin.x();
        matrix(1, 3) = origin.y();
        matrix(2, 3) = origin.z();

        auto rotation = tf.getRotation();
        Eigen::Quaternionf q(rotation.w(), rotation.x(), rotation.y(), rotation.z());
        matrix.block<3, 3>(0, 0) = q.toRotationMatrix();

        return matrix;
    }

    // Publish the Map
    void publishMap() {
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*map_, map_msg);
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = this->get_clock()->now();
        map_publisher_->publish(map_msg);
        RCLCPP_INFO(this->get_logger(), "Published updated map.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPMappingNode>());
    rclcpp::shutdown();
    return 0;
}
