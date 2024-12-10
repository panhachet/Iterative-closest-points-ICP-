#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "iostream"
#include "vector"
#include "icp.hpp"


using namespace std;

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

class icp_node : public rclcpp::Node 
{
    public:
    icp_node(): Node("icp_node"), 
    icp_(vector<vector<double>>{}, vector<vector<double>>{},
    {0.0, 0.0, 0.0}, 1000, 1e-6)
    {
        rclcpp::QoS qos_profile(rclcpp::KeepLast(100));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", qos_profile,
            std::bind(&icp_node::laser_callback, this, std::placeholders::_1)
        );
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
            std::bind(&icp_node::odom_callback, this, std::placeholders::_1)
        );

        new_laser_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("new_scan", 10);
    }

    private:

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            if(first_move)
            {
                prev_robot_pose[0]  = msg->pose.pose.position.x;
                prev_robot_pose[1] = msg->pose.pose.position.y;
                auto q = msg->pose.pose.orientation;
                prev_robot_pose[2]  = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y  + q.z*q.z));
                first_move = false;
            }
            robot_pose[0]  = msg->pose.pose.position.x;
            robot_pose[1] = msg->pose.pose.position.y;
            auto q = msg->pose.pose.orientation;
            robot_pose[2]  = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y  + q.z*q.z));

        }

        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            int downsample_factor = 10; 
            obj_x.clear();
            obj_y.clear();
            for(size_t i =0; i<msg->ranges.size(); i+=downsample_factor)
            {
                float range = msg->ranges[i];
                if(range < msg->range_min || range > msg->range_max)
                {
                    continue;
                }

                float angle =  msg->angle_min + i * msg->angle_increment;
                float x = range * cos(angle);
                float y = range * sin(angle);
                float obx = x * cos(robot_pose[2]) - y * sin(robot_pose[2]);
                float oby = x * sin(robot_pose[2]) + y * cos(robot_pose[2]);
                obj_x.push_back(x);
                obj_y.push_back(y);
            }
            obj =  icp_.combine_data(obj_x, obj_y);
            round_store_Q(obj);

            if(flag == 0)
            {
                flag =  1;
                if (Q.empty())
                {
                    Q =  obj;
                    RCLCPP_WARN(this->get_logger(), "No points available for marker visualization.");
                    flag = 0;
                }
                else
                {
                    P = obj;
                    icp_.set_P(P);
                    icp_.set_Q(Q);
                    dx = robot_pose[0] - prev_robot_pose[0];
                    dy = robot_pose[1] - prev_robot_pose[1];
                    dtheta = robot_pose[2] - prev_robot_pose[2];
                    icp_.set_X({dx,dy,dtheta});
                    auto [aligned, dx1, conv] = icp_.icp_function();
                    cout << "conv" <<  conv << endl;
                    cout << Q.size() << endl;
                    cout << dx << "\t" << dy << "\t" << dtheta << endl;
                    if (conv)
                    {
                        prev_robot_pose = robot_pose;
                        Q.insert(Q.end(), aligned.begin(), aligned.end());
                        round_store_Q(Q);
                        sort(Q.begin(), Q.end());
                        Q.erase(unique(Q.begin(), Q.end()), Q.end());
                    }
                    auto new_scan = sensor_msgs::msg::LaserScan();
                        new_scan.header.frame_id = msg->header.frame_id;
                        new_scan.header.stamp = this->get_clock()->now();
                        new_scan.angle_min = msg->angle_min;
                        new_scan.angle_max = msg->angle_max;
                        new_scan.angle_increment = msg->angle_increment * downsample_factor;
                        new_scan.time_increment = msg->time_increment * downsample_factor;
                        new_scan.range_min = msg->range_min;
                        new_scan.range_max = msg->range_max;
                        new_scan.ranges.resize(Q.size(), new_scan.range_max);
                        for (size_t i = 0; i < Q.size(); i += downsample_factor)
                        {
                            double x = Q[i][0];
                            double y = Q[i][1];
                            double range = std::sqrt(x * x + y * y);
                            double angle = std::atan2(y, x);
                            int index = (angle - new_scan.angle_min) / new_scan.angle_increment;
                            if (index >= 0 && index < new_scan.ranges.size())
                            {
                                new_scan.ranges[index] = range;
                            }
                        }
                        new_laser_pub->publish(new_scan);
                    flag = 0;
                }
                robot_move = false;
                }
        }

 

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr new_laser_pub;
        icp icp_;

        vector<double> robot_pose = {0.0, 0.0, 0.0};
        vector<double> prev_robot_pose = {0.0, 0.0, 0.0};
        vector<vector<double>> obj;
        vector<vector<double>> P;
        vector<vector<double>> Q;
        vector<double> obj_x;
        vector<double> obj_y;
        int flag = 0;
        bool robot_move = false;
        bool first_move = true;
        double dx;
        double dy;
        double dtheta;


};

int main(int argc,  char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<icp_node>());
    rclcpp::shutdown();
    return 0;
}