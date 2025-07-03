from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpu6050driver',
            executable='mpu6050driver',
            name='mpu6050_node',
            output='screen'
        ),

        Node(
            package='my_robot_description',
            executable='wheel_odometry_publisher',
            name='wheel_odometry',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('my_robot_description'),
                    'launch',
                    'control_launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ldlidar_stl_ros2'),
                    'launch',
                    'ld06.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('my_robot_description'),
                    'launch',
                    'tf_publisher.launch.py')
            )
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/root/ekf.yaml']
        ),

        Node(
            package='motor_encoder',
            executable='motor_encoder_pub',
            name='encoder_publisher',
            output='screen'
        ),

        Node(
            package='motor_control',
            executable='listener',
            name='motor_listener',
            output='screen'
        ),
        # Camera
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='motor_control',
            executable='talker',
            name='motor_talker',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_async_launch.py')
            ),
            launch_arguments={'params_file': '/slam_params.yaml'}.items()
        ),
    ])
