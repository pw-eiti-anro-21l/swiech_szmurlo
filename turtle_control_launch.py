import os
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node

def generate_launch_description():
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'
    return LaunchDescription([
        Node(
            package='turtle_control',
            executable='turtle_control_talker',
            name='custom_parameter_node',
            prefix = "gnome-terminal --", 
            # env = proc_env,
            output='screen',
            emulate_tty=True,
            parameters=[
                {'up_key': 'w'},
                {'down_key': 's'},
                {'left_key': 'a'},
                {'right_key': 'd'}
            ]
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
        ),
    ])
