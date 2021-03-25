import os
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'
    config = os.path.join('src','swiech_szmurlo','turtle_control','config', 'turtle_key_control_node.yaml')
    return LaunchDescription([
        Node(
            package='turtle_control',
            executable='turtle_control_talker',
            name='turtle_key_control_node',
            prefix = "gnome-terminal --", 
            # env = proc_env,
            output='screen',
            emulate_tty=True,
            parameters=[config]
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
        ),
    ])
