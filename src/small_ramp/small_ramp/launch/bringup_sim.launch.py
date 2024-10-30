import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    # Package directory
    pkg_share = get_package_share_directory('small_ramp')
    world_file = os.path.join(pkg_share, 'worlds', 'rampRider.sdf')
    robot_description_file = os.path.join(pkg_share, 'urdf', '4wd_robot.urdf')

    return LaunchDescription([
        # Robot description argument
        DeclareLaunchArgument(
            'robot_description', 
            default_value=Command(['xacro ', robot_description_file])
        ),

        # Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
            )
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('robot_description')}],
        ),

        # Teleop Twist Keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[('/cmd_vel', '/robot/cmd_vel')]
        ),
    ])
