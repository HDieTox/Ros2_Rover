from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='',
        description='Chemin vers le fichier mission .plan (QGroundControl)'
    )

    pkg_share = get_package_share_directory('rover_autonomous')
    ekf_param_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    navigation_node = Node(
        package='rover_autonomous',          # à remplacer par le nom de votre package
        executable='nav_controller',
        name='nav_controller',
        parameters=[{'mission_file': LaunchConfiguration('mission_file')}]
    )

    serial_stm_node = Node(
        package='rover_autonomous',
        executable='serial_stm',
        name='serial_stm'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param_file]
    )


    return LaunchDescription([
        mission_file_arg,
        navigation_node,
        serial_stm_node,
        ekf_node,
    ])