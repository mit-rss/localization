import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    localization_params = os.path.join(
        get_package_share_directory('localization'),
        'test_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='localization',
            executable='motion_model_test',
            name='particle_filter',
            output='screen',
            parameters=[localization_params]
        ),
    ])
