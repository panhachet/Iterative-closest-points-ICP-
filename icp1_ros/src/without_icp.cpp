#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "iostream"
#include "vector"
#include "icp.hpp"
#include "KDTree.hpp" 

using namespace  std;

double euclidean_distance(const std::vector<double>& p, const std::vector<double>& q) {
    double sum = 0.0;
    for (size_t i = 0; i < p.size(); ++i) {
        sum += (p[i] - q[i]) * (p[i] - q[i]);
    }
    return std::sqrt(sum);
}

double round_to_two_decimal_places(double value)
{
    return std::round(value * 100.0) / 100.0;
}

void round_store_Q(vector<vector<double>>& store_Q) {
    for (auto& point : store_Q) {
        point[0] = round_to_two_decimal_places(point[0]);  
        point[1] = round_to_two_decimal_places(point[1]);  
    }
}


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

        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", qos,
            std::bind(&icp_node::imu_callback, this, std::placeholders::_1)
        );

        pcq_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "map_point", 10
        );

        pcp_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "p_point", 10
        );

        icp_timer = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&icp_node::icp_callback, this)
        );


    }

    private:

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            auto q = msg->orientation;
            yaw = atan2(2.0*(q.w*q.z + q.x*q.y),(1.0 - 2.0*(q.y*q.y  + q.z*q.z)));
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            if (first_move)
            {
                prev_pos[0] = msg->pose.pose.position.x;
                prev_pos[1] = msg->pose.pose.position.y;
                auto q = msg->pose.pose.orientation;
                prev_pos[2] =  yaw;
                first_move  = false;
            }
            else
            {
                curr_pos[0] = msg->pose.pose.position.x;
                curr_pos[1] = msg->pose.pose.position.y;
                auto q = msg->pose.pose.orientation;
                curr_pos[2] = yaw;
            }
        }

        void publish_p_pointcloud(const vector<vector<double>> &P)
        {
            auto pc_msg = sensor_msgs::msg::PointCloud2();
            pc_msg.header.frame_id = "map";
            pc_msg.header.stamp = this->get_clock()->now();

            pc_msg.height = 1;
            pc_msg.width = P.size();
            pc_msg.is_dense = true;
            pc_msg.is_bigendian = false;

            sensor_msgs::PointCloud2Modifier modidier(pc_msg);
            modidier.setPointCloud2FieldsByString(1, "xyz");
            modidier.resize(P.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");

            for (const auto &point : P) {
                *iter_x = static_cast<float>(point[0]);  
                *iter_y = static_cast<float>(point[1]); 
                *iter_z = 0.0f;                          

                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
            pcp_pub->publish(pc_msg);
        }

        void publish_q_pointcloud(const vector<vector<double>> &Q)
        {
            auto qpc_msg = sensor_msgs::msg::PointCloud2();
            qpc_msg.header.frame_id = "map";
            qpc_msg.header.stamp = this->get_clock()->now();

            qpc_msg.height = 1;
            qpc_msg.width = Q.size();
            qpc_msg.is_dense = true;
            qpc_msg.is_bigendian = false;

            sensor_msgs::PointCloud2Modifier modidier(qpc_msg);
            modidier.setPointCloud2FieldsByString(1, "xyz");
            modidier.resize(Q.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x(qpc_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(qpc_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(qpc_msg, "z");

            for (const auto &point : Q) {
                *iter_x = static_cast<float>(point[0]);  
                *iter_y = static_cast<float>(point[1]); 
                *iter_z = 0.05;                          

                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
            pcq_pub->publish(qpc_msg);
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
                float obx = x * cos(curr_pos[2]) - y * sin(curr_pos[2]) + curr_pos[0];
                float oby = x * sin(curr_pos[2]) + y * cos(curr_pos[2])  + curr_pos[1];
                obj_x.push_back(obx);
                obj_y.push_back(oby);
            }
            obj =  icp_.combine_data(obj_x, obj_y);
            if (flag==0)
            {
                if(Q.empty())
                {
                    data = obj;
                    Q = data; 
                    flag = 0;
                }
                else
                {
                    P = obj;
                    closest_points.clear();
                    closest_points = find_closest_points_kdtree_2d(data, P);
                    round_store_Q(closest_points);
                    sort(closest_points.begin(), closest_points.end());
                    closest_points.erase(unique(closest_points.begin(), closest_points.end()), closest_points.end());
                    P_size = P.size();
                    C_size = closest_points.size();
                    // cout << "================" << endl;
                    // cout << P.size() << endl;
                    // cout << closest_points.size() << endl;
                    // cout << "================" << endl;
                    publish_q_pointcloud(Q);
                    publish_p_pointcloud(closest_points);
                    data.insert(data.end(), P.begin(), P.end());
                    round_store_Q(data);
                    sort(data.begin(), data.end());
                    data.erase(unique(data.begin(), data.end()), data.end());
                    prev_pos = curr_pos;
                    flag = 0;
                }
            }

        }

    void icp_callback()
    {
        dx = curr_pos[0] - prev_pos[0];
        dy = curr_pos[1] - prev_pos[1];
        dyaw = curr_pos[2] - prev_pos[2];
        // cout << "================" << endl;
        // cout << "dx" << dx << "\t" << dy << "\t" << dyaw << endl;
        // cout << "================" << endl;
        if  (dx <= 0.000001  && dy <= 0.000001  && dyaw <= 0.000001 )
        {

            publish_p_pointcloud(P);
            Q.insert(Q.end(), P.begin(), P.end());
            round_store_Q(Q);
            sort(Q.begin(), Q.end());
            Q.erase(unique(Q.begin(), Q.end()), Q.end());
            publish_q_pointcloud(Q);

        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcq_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcp_pub;
    rclcpp::TimerBase::SharedPtr icp_timer;
    icp icp_;

    vector<double> prev_pos = {0.0, 0.0, 0.0};
    vector<double> curr_pos = {0.0, 0.0, 0.0};
    vector<double> obj_x;
    vector<double> obj_y;
    vector<vector<double>> obj;
    vector<vector<double>> data;
    vector<vector<double>> Q;
    vector<vector<double>> P;
    vector<vector<double>> closest_points;
    bool first_move = true;
    int flag = 0;
    int icp_flag = 0;
    double dx;
    double dy;
    double dyaw;
    double yaw = 0.0;

    int P_size;
    int C_size;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<icp_node>());
    rclcpp::shutdown();
    return 0;
}