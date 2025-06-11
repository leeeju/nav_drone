import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodes launching commands
    costmap_publisher = Node(
        package='nav2_drone_costmap_3d',
        executable='costmap_publisher',
        name='costmap_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'map_frame': 'map'},
            {'drone_base_frame': 'base_link'},
            {'transform_tolerance': 0.1},
            {'lookahead_dist': 2.0},
            {'min_lookahead_dist': 0.8},
            {'max_lookahead_dist': 4.0},
            {'lookahead_time': 1.5},
            {'use_velocity_scaled_lookahead_dist': False},
            {'drone_radius': 0.5},
            {'safety_radius': 0.3}
        ]
    )

    # Launch description
    ld = LaunchDescription()
    ld.add_action(costmap_publisher)

    return ld
