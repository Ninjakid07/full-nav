from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    aiil_rosbot_demo_dir = get_package_share_directory('aiil_rosbot_demo')

    return LaunchDescription([
        # 1. Start find_object_2d_robot first
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(aiil_rosbot_demo_dir, 'launch', 'find_object_2d_robot.launch.py')
            ),
            launch_arguments={'gui': 'false'}.items()
        ),

        # 2. Return Navigation Nodes (path_rec and rev_waypoint)
        Node(
            package='par_1',
            executable='path_rec',
            name='path_recorder',
            output='screen',
            emulate_tty=True,
        ),
        
        Node(
            package='par_1',
            executable='rev_waypoint',
            name='waypoint_follower',
            output='screen',
            emulate_tty=True,
        ),
        
        # 3. Hazard Detection Node
        Node(
            package='par_1',
            executable='hazard_detection_node',
            name='hazard_detector',
            output='screen',
            emulate_tty=True,
        ),
        
        # 4. Wall Follower Node
        Node(
            package='par_1',
            executable='wall_follower',
            name='wall_follower',
            output='screen',
            emulate_tty=True,
        )
    ])