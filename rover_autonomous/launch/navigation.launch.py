from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Contrôleur principal
        Node(
            package='rover_autonomous',
            executable='main_controller',
            name='navigation_controller',
            parameters=[{'mission_file': 'src/SQUARE.plan'}]
        ),
        
        # Convertisseur PPM
        Node(
            package='rover_autonomous',
            executable='ppm_converter',
            name='ppm_interface'
        )
    ])