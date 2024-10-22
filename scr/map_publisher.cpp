#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <vector>
#include <cmath>

class MapPublisherNode : public rclcpp::Node
{
public:
  MapPublisherNode()
    : Node("map_publisher"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
  {
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&MapPublisherNode::scan_callback, this, std::placeholders::_1));

    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/accumulated_points", 10);

    map_.header.frame_id = "map";
    map_.info.resolution = 0.1; 
    map_.info.width = 100;       
    map_.info.height = 100;      
    map_.info.origin.position.x = -5.0; 
    map_.info.origin.position.y = -5.0;
    map_.data.resize(map_.info.width * map_.info.height, -1); 
  }

private:
  std::vector<geometry_msgs::msg::Point> accumulated_points_;  // Array para almacenar los puntos transformados

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      // Obtener la transformación del marco base_scan a map usando ICP
      transform_stamped = tf_buffer_.lookupTransform("map", "base_scan", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Couldn't transform base_scan to map: %s", ex.what());
      return;
    }

    std::vector<geometry_msgs::msg::Point> new_points;  // Array para almacenar los puntos del nuevo escaneo

    // Transformar los puntos del escaneo actual
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float range = msg->ranges[i];
      if (range < msg->range_min || range > msg->range_max) {
        continue;
      }

      float angle = msg->angle_min + i * msg->angle_increment;
      float x = range * cos(angle);
      float y = range * sin(angle);

      geometry_msgs::msg::PointStamped scan_point, map_point;
      scan_point.header.frame_id = "base_scan";
      scan_point.point.x = x;
      scan_point.point.y = y;
      scan_point.point.z = 0.0;

      // Transforma los puntos escaneados actuales a la referencia global del mapa usando ICP
      try {
        tf2::doTransform(scan_point, map_point, transform_stamped);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Couldn't transform the point: %s", ex.what());
        continue;
      }

      new_points.push_back(map_point.point);  // Añadir puntos transformados al array
    }

    // Agregar los puntos nuevos al array acumulado sin eliminar los puntos previos
    accumulated_points_.insert(accumulated_points_.end(), new_points.begin(), new_points.end());

    // Crear el mapa basado en los puntos acumulados
    update_map_with_accumulated_points();

    // Publicar el PointCloud de los puntos acumulados
    publish_accumulated_points();
  }

  void update_map_with_accumulated_points()
  {
    int map_center_x = map_.info.width / 2;
    int map_center_y = map_.info.height / 2;

    // Limpiar el mapa antes de actualizarlo
    std::fill(map_.data.begin(), map_.data.end(), -1);

    // Iterar sobre todos los puntos acumulados y actualizar el mapa de ocupación
    for (const auto& point : accumulated_points_) {
      int map_x = static_cast<int>((point.x / map_.info.resolution) + map_center_x);
      int map_y = static_cast<int>((point.y / map_.info.resolution) + map_center_y);

      if (map_x >= 0 && map_x < static_cast<int>(map_.info.width) && 
          map_y >= 0 && map_y < static_cast<int>(map_.info.height)) 
      {
        // Marcar las celdas ocupadas en el mapa
        map_.data[map_y * map_.info.width + map_x] = 100;
      }
    }

    // Publicar el mapa actualizado
    map_.header.stamp = this->now();
    map_publisher_->publish(map_);
  }

  void publish_accumulated_points()
  {
    // Crear un mensaje PointCloud2
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = this->now();

    cloud_msg.height = 1;  // 1 línea de puntos
    cloud_msg.width = accumulated_points_.size();  // Número de puntos acumulados

    // Definir los campos x, y, z para PointCloud2
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(accumulated_points_.size());

    // Llenar los datos de la nube de puntos
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto& point : accumulated_points_) {
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    // Publicar el PointCloud2
    pointcloud_publisher_->publish(cloud_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  nav_msgs::msg::OccupancyGrid map_;  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
