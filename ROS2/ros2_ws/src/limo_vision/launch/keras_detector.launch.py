from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limo_vision',
            executable='keras_detector_v2',
            name='keras_detector_v2',
            output='screen'
        )
    ])
