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
    return std::round(value * 1000.0) / 1000.0;
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

        new_laser_pub = this->create_publisher<visualization_msgs::msg::Marker>("new_scan", 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&icp_node::timer_callback, this));
    }

    private:

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            robot_pose[0]  = msg->pose.pose.position.x;
            robot_pose[1] = msg->pose.pose.position.y;
            auto q = msg->pose.pose.orientation;
            robot_pose[2]  = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y  + q.z*q.z));
            dx = robot_pose[0] - prev_robot_pose[0];
            dy = robot_pose[1] - prev_robot_pose[1];
            dtheta = robot_pose[2] - prev_robot_pose[2];
            double distance_moved = std::sqrt(dx * dx + dy * dy);
            double angle_moved = std::fabs(dtheta);
            if (distance_moved > 0.005 || angle_moved > 0.01)
            {
                robot_move = true;
            }
            else
            {
                robot_move = false;
            }
            prev_robot_pose = robot_pose;

        }

        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            obj_x.clear();
            obj_y.clear();
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
                float obx = x * cos(robot_pose[2]) - y * sin(robot_pose[2]);
                float oby = x * sin(robot_pose[2]) + y * cos(robot_pose[2]);
                obj_x.push_back(x);
                obj_y.push_back(y);
            }
            obj =  icp_.combine_data(obj_x, obj_y);
            round_store_Q(obj);
        }

        void timer_callback()
        {
            if(!robot_move)
            {
                return;
            }
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
                    icp_.set_X({dx,dy,dtheta});
                    auto [aligned, dx1, conv] = icp_.icp_function();
                    cout << "conv" <<  conv << endl;
                    cout << Q.size() << endl;
                    if (conv)
                    {
                        Q.insert(Q.end(), aligned.begin(), aligned.end());
                        round_store_Q(Q);
                        sort(Q.begin(), Q.end());
                        Q.erase(unique(Q.begin(), Q.end()), Q.end());
                        auto marker =  visualization_msgs::msg::Marker();
                        marker.header.frame_id = "map";
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
                        
                        for (const  auto &point : Q)
                        {
                            geometry_msgs::msg::Point p;
                            p.x = point[0];
                            p.y = point[1];
                            p.z = 0.0;
                            marker.points.push_back(p);
                        }
                        new_laser_pub->publish(marker);
                    }
                    flag = 0;
                }
                robot_move = false;
                }
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr new_laser_pub;
        rclcpp::TimerBase::SharedPtr timer;
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