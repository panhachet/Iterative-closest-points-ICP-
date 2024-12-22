#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "icp.hpp"
#include <vector>
#include <cmath>

class ICP_Mapping : public rclcpp::Node 
{

public:
    ICP_Mapping() 
    :   Node("icp_node"),
        icp_(std::vector<std::vector<double>>{},
        std::vector<std::vector<double>>{},
        {0.0, 0.0, 0.0}, 100, 1e-6)
    {
        rclcpp::QoS qos(rclcpp::KeepLast(100));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", qos, std::bind(&ICP_Mapping::scan_callback, this, std::placeholders::_1)
        );

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&ICP_Mapping::odom_callback, this, std::placeholders::_1)
        );

        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", qos, std::bind(&ICP_Mapping::imu_callback, this, std::placeholders::_1)
        );

        //map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    }

private:

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
            msg->orientation.w
        );
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        X[2] = yaw;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<double> data_x, data_y;
        for (size_t i = 0; i < msg->ranges.size(); ++i) 
        {
            float range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max) {
                continue;
            }
            float angle = msg->angle_min + i * msg->angle_increment;
            float x_ = range * cos(angle);
            float y_ = range * sin(angle);
            float x1 = cos(X[2]) * x_ - sin(X[2]) * y_;
            float y1 = sin(X[2]) * x_ + cos(X[2]) * y_;

            data_x.push_back(x1);
            data_y.push_back(y1);
            cout << "ok" << endl;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
    
    std::vector<std::vector<double>> New_P;
    std::vector<std::vector<double>> P;
    std::vector<std::vector<double>> Q;
    std::vector<double> data_x;
    std::vector<double> data_y;
    std::vector<double> X = {0.0, 0.0, 0.0};

    icp icp_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICP_Mapping>());
    rclcpp::shutdown();
    return 0;
}