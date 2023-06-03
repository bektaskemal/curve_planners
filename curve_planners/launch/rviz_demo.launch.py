from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare the launch argument for the params.yaml file
    default_params_file = PathJoinSubstitution(substitutions=[get_package_share_directory("curve_planners"), "params", "params.yaml"])
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the params file'
    )

    rviz_config = PathJoinSubstitution(substitutions=[get_package_share_directory("curve_planners"), "rviz", "wp_demo.rviz"])

    return LaunchDescription([
        params_file,
        Node(
            package='curve_planners',
            executable='curve_planners_node',
            name='curve_planners_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        ),
        Node(
            package='curve_planners',
            executable='rviz_relay_wp.py',
            name='rviz_relay_wp',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
