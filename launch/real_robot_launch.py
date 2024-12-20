"""
Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
import os.path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_robot_driver_unitree_z1',
            executable='sas_robot_driver_unitree_z1_node',
            name='unitree_z1_1',
            parameters=[{
                "gripper_attached": "true",
                "mode": "PositionControl",
                "verbosity": "true",
                "joint_limits_min": [-150.0, 0.0, -160.0, -80.0, -80.0, -160.0],  # The last joint has no limit
                "joint_limits_max": [150.0, 180.0, 0.0, 80.0, 80.0, 160.0],  # The last joint has no limit
                "thread_sampling_time_sec": 0.002 # Robot thread is at 500 Hz
            }]
        ),

    ])
