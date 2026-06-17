import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('nav_failsafe')
    params_file = os.path.join(package_dir, 'config', 'nav2_params.yaml')

    # 🔌 Command Velocity to PX4 Bridge Node
    bridge_node = Node(
        package='nav_failsafe',
        executable='cmd_vel_to_px4_bridge_node',
        name='cmd_vel_to_px4_bridge_node',
        output='screen'
    )

    # 🧠 Nav2 Planner Server Node (SmacPlanner3D ko load karne ke liye)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        bridge_node,
        planner_server
    ])
