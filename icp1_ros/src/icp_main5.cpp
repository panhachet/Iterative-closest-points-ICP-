#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "iostream"
#include "vector"
#include "icp.hpp"

using namespace  std;

class icp_node : public rclcpp::Node
{
    public:
    icp_node(): Node("icp_node"),
    icp_(vector<vector<double>>{}, vector<vector<double>>{},
    {0.0, 0.0, 0.0}, 1000, 1e-6)
    {
        rclcpp::QoS qos(rclcpp::KeepLast(100));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", qos,
            std::bind(&icp_node::laser_callback, this, std::placeholders::_1)
        );

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&icp_node::odom_callback, this, std::placeholders::_1)
        );

        new_laser_pub = this->create_publisher<visualization_msgs::msg::Marker>(
            "new_scan", 10
        );
    }

    private:

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            if (first_move)
            {
                prev_pos[0] = msg->pose.pose.position.x;
                prev_pos[1] = msg->pose.pose.position.y;
                auto q = msg->pose.pose.orientation;
                prev_pos[2] =  atan2(2.0*(q.w*q.z + q.x*q.y),(1.0 - 2.0*(q.y*q.y  + q.z*q.z)));
                first_move  = false;
            }
            else
            {
                curr_pos[0] = msg->pose.pose.position.x;
                curr_pos[1] = msg->pose.pose.position.y;
                auto q = msg->pose.pose.orientation;
                curr_pos[2] = atan2(2.0*(q.w*q.z + q.x*q.y),(1.0 - 2.0*(q.y*q.y  + q.z*q.z)));
            }
        }

        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            obj_x.clear();
            obj_y.clear();
            dx = curr_pos[0] - prev_pos[0];
            dy = curr_pos[1] - prev_pos[1];
            dyaw = curr_pos[2] - prev_pos[2];
            if  (dx >= 0.0001  || dy >= 0.0001  || dyaw >= 0.0001 )
            {
            prev_pos = curr_pos;
            for(size_t i =0; i<msg->ranges.size(); i++)
            {
                float range = msg->ranges[i];
                if(range < msg->range_min || range > msg->range_max)
                {
                    continue;
                }

                float angle =  msg->angle_min + i * msg->angle_increment;
                float x = range * cos(angle);
                float y = range * sin(angle);
                float obx = x * cos(curr_pos[2]) - y * sin(curr_pos[2]) + curr_pos[0];
                float oby = x * sin(curr_pos[2]) + y * cos(curr_pos[2]) + curr_pos[1];
                obj_x.push_back(obx);
                obj_y.push_back(oby);
            }
            obj =  icp_.combine_data(obj_x, obj_y);
            data.insert(data.end(), obj.begin(), obj.end());
            auto marker =  visualization_msgs::msg::Marker();
            marker.header.frame_id = "odom";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "scan_points";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.05;  
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0;  
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            cout << data.size() << endl;
            for (const  auto &point : data)
            {
                geometry_msgs::msg::Point p;
                p.x = point[0];
                p.y = point[1];
                p.z = 0.0;
                marker.points.push_back(p);
            }
            new_laser_pub->publish(marker);
            
            }
        }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr new_laser_pub;
    icp icp_;

    vector<double> prev_pos = {0.0, 0.0, 0.0};
    vector<double> curr_pos = {0.0, 0.0, 0.0};
    vector<double> obj_x;
    vector<double> obj_y;
    vector<vector<double>> obj;
    vector<vector<double>> data;
    bool first_move = true;
    double dx;
    double dy;
    double dyaw;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<icp_node>());
    rclcpp::shutdown();
    return 0;
}