cmake_minimum_required(VERSION 3.8)
project(icp1_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)

include_directories(include/icp1_ros)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(map src/icp_map.cpp)
ament_target_dependencies(map rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros visualization_msgs)

add_executable(icp1 src/icp_main2.cpp src/icp.cpp) 
ament_target_dependencies(icp1 rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros visualization_msgs)


add_executable(icp3 src/icp_main3.cpp src/icp.cpp) 
ament_target_dependencies(icp3  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros visualization_msgs)

add_executable(icp4 src/icp_main4.cpp src/icp.cpp) 
ament_target_dependencies(icp4  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros visualization_msgs)

add_executable(icp5 src/icp_main5.cpp src/icp.cpp) 
ament_target_dependencies(icp5  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros visualization_msgs)


add_executable(pc src/point_cloud.cpp) 
ament_target_dependencies(pc  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros visualization_msgs laser_geometry)


install(TARGETS
  map
  icp1
  icp3
  icp4
  icp5
  pc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
