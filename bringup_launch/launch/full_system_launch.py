from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld06_launch_path = os.path.join(
        get_package_share_directory('ldlidar_stl_ros2'),
        'launch',
        'ld06.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Motor Control
        Node(
            package='motor_control',
            executable='talker',
            name='motor_control_talker',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='motor_control',
            executable='listener',
            name='motor_control_listener',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Motor Encoder
        Node(
            package='motor_encoder_ros2',
            executable='talker',
            name='motor_encoder_talker',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='motor_encoder_ros2',
            executable='listener',
            name='motor_encoder_listener',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # IMU
        Node(
            package='mpu6050driver',
            executable='mpu6050driver',
            name='mpu6050_node',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # LIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld06_launch_path),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Camera
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['/path/to/your_robot.urdf']  # <<< UPDATE this
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='encoder_odometry',
            executable='odometry_subscriber',
            name='encoder_odometry',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
