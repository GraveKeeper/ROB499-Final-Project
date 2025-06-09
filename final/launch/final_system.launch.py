from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('final'),
                'launch',
                'gazebo_with_camera.launch.py'
            )
        )
    )
        
        Node(
            package='final',
            executable='patrol_node',
            name='patrol_node',
            output='screen',
            parameters=[{'waypoint_file': '/absolute/path/to/waypoints.yaml'}],
        ),
        Node(
            package='final',
            executable='detector_node',
            name='detector_node',
            output='screen',
        ),
        Node(
            package='final',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),
        Node(
            package='final',
            executable='control_node',
            name='control_node',
            output='screen',
        ),
        Node(
            package='rviz2', 
            executable='rviz2', 
            name='rviz2', 
            arguments=['-d', 'rviz/final_view.rviz']
        ),
        return LaunchDescription([
        gazebo_launch,
        detector_node,
    ])