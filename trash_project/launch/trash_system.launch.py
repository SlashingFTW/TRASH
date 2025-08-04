from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trash_project',
            executable='canny_node',
            name='cv_node',
            output='screen'
        ),
        Node(
            package='trash_project',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),
        Node(
            package='trash_project',
            executable='spectrometer_node',
            name='spectrometer_node',
            output='screen'
        ),
        Node(
            package='trash_project',
            executable='gui_node',
            name='gui_node',
            output='screen'
        ),
    ])
