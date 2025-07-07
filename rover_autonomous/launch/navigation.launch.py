from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Contrôleur principal
        Node(
            package='autonomous_navigation',
            executable='main_controller',
            name='navigation_controller',
            parameters=[{'mission_file': '/chemin/vers/mission.plan'}]
        ),
        
        # Convertisseur PPM
        Node(
            package='autonomous_navigation',
            executable='ppm_converter',
            name='ppm_interface'
        )
    ])
p