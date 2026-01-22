from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    objects_tracker_node = Node(
        package='lidar_objects_tracker',
        executable='objects_tracker_node',
        name='objects_tracker_node',
        output='screen',
        remappings=[
            ('scan', 'lidar/base/front/scan'),
        ],
        parameters=[
            {'cluster_neighbor_radius': 0.5},
            {'cluster_min_points': 10},
            {'cluster_max_points': 10000},
            {'birth_existence_prob': 0.05},
            {'survival_prob': 0.8},
            {'detection_prob': 0.8},
            {'kf_acc_uncertainty': 0.1},
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_objects_tracker',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['objects_tracker_node']},
        ],
    )

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', '/home/ubuntu/ros2-service-robot/workspace/src/lidar_objects_tracker/rosbag2_sensors'
        ],
        output='screen'
    )

    return LaunchDescription([
        objects_tracker_node,
        lifecycle_manager,
        bag_play,
    ])
