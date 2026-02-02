from setuptools import setup

package_name = 'mary_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@mail.utoronto.ca',
    description='Control package for MARY drone - person following, altitude hold, and flight control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'comm_node = mary_control.comm_node:main',
            'follower_node = mary_control.follower_node:main',
            'altitude_controller_node = mary_control.altitude_controller_node:main',
            'mission_manager_node = mary_control.mission_manager_node:main',
        ],
    },
)
