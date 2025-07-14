from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Serial Receiver
        Node(
            package='rover_gpsreceiver',
            executable='serial_nmea_publisher',
            name='Serial_NMEA',
        ),
        
        # Convertisseur PPM
        Node(
            package='rover_gpsreceiver',
            executable='nmea_parser',
            name='nmea_parser'
        )
    ])