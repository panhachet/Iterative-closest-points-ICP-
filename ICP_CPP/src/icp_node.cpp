#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "ICP.hpp" // Incluye tu clase de ICP
#include <Eigen/Dense>

using std::placeholders::_1;
using namespace Eigen;

class IcpNode : public rclcpp::Node
{
public:
    IcpNode() : Node("icp_node")
    {
        // Suscriptor de datos del LiDAR
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&IcpNode::lidar_callback, this, _1));

        // Publicador de mapas
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("icp_map", 10);

        RCLCPP_INFO(this->get_logger(), "ICP Node initialized.");
    }

private:
    // Callback para recibir los datos del LiDAR
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<double> lidar_data_x, lidar_data_y;

        // Convierte los datos del LiDAR en coordenadas cartesianas (x, y)
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (std::isfinite(msg->ranges[i])) // Filtra los valores no válidos
            {
                double angle = msg->angle_min + i * msg->angle_increment;
                double x = msg->ranges[i] * cos(angle);
                double y = msg->ranges[i] * sin(angle);
                lidar_data_x.push_back(x);
                lidar_data_y.push_back(y);
            }
        }

        // Utiliza la clase ICP para crear el mapa
        std::vector<vector<double>> P = icp::combine_data(lidar_data_x, lidar_data_y);
        std::vector<vector<double>> Q = previous_scan_; // Escaneo anterior
        icp icp_instance(P, Q, {0.0, 0.0, 0.0}, 100, 0.001);

        // Ejecuta la función ICP
        auto [new_scan, pose] = icp_instance.icp_function();
        previous_scan_ = new_scan;

        // Publica el mapa generado
        publish_map(pose);
    }

    // Publica el mapa usando un OccupancyGrid de ROS2
    void publish_map(const std::vector<double>& pose)
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "map";

        // Configura los parámetros del OccupancyGrid
        map_msg.info.resolution = 0.05; // Resolución del mapa en metros
        map_msg.info.width = 100;       // Ancho del mapa
        map_msg.info.height = 100;      // Altura del mapa
        map_msg.info.origin.position.x = pose[0];
        map_msg.info.origin.position.y = pose[1];
        map_msg.info.origin.position.z = 0.0;
        map_msg.info.origin.orientation.w = cos(pose[2] / 2);
        map_msg.info.origin.orientation.z = sin(pose[2] / 2);

        // Inicializa los datos del mapa (simplemente como un ejemplo)
        map_msg.data.resize(map_msg.info.width * map_msg.info.height);
        std::fill(map_msg.data.begin(), map_msg.data.end(), -1); // -1 indica desconocido

        map_publisher_->publish(map_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    std::vector<vector<double>> previous_scan_; // Guarda el escaneo anterior
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IcpNode>());
    rclcpp::shutdown();
    return 0;
}
