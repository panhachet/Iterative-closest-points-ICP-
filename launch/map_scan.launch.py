import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    turtlebot3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch'
    )
    

    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'rviz',
        'model.rviz'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_launch_dir, 'turtlebot3_world.launch.py')
            )
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen',
            parameters=[{'use_sim_time': True}]  
        ),


        Node(
            package='project',
            executable='icp_node', 
            name='icp_node',
            output='screen',
            parameters=[{'use_sim_time': True}]  
        ),


        Node(
            package='project',
            executable='map_publisher', 
            name='map_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}] 
        ),
    ])
