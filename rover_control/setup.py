from setuptools import setup

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tom',
    maintainer_email='tom.compagnon@ensea.fr',
    description='ROS2 driver for robot motor control and PWM decoding',
    entry_points={
        'console_scripts': [
            'pwn_decoder = rover_control.pwn_decoder:main',
        ],
    },
)
