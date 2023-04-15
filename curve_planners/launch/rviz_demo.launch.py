
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
# ROS2 launch file that runs curve_planners_node, rviz_relay_wp.py and rviz2
# Start rviz2 with the config file in the curve_planners/rviz/wp_demo.rviz

def generate_launch_description():
    rviz_config = PathJoinSubstitution(substitutions=[get_package_share_directory("curve_planners"), "rviz", "wp_demo.rviz"])
    return LaunchDescription([
        Node(
            package='curve_planners',
            executable='curve_planners_node',
            name='curve_planners_node',
            output='screen',
        ),
        Node(
            package='curve_planners',
            executable='rviz_relay_wp.py',
            name='rviz_relay_wp',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])