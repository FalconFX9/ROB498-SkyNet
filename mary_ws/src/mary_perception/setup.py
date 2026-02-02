from setuptools import setup

package_name = 'mary_perception'

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
    description='Perception package for MARY drone - person detection, tracking, and ToF processing',
    license='MIT',
    entry_points={
        'console_scripts': [
            't265_pose_node = mary_perception.t265_pose_node:main',
            'person_tracker_node = mary_perception.person_tracker_node:main',
            'tof_processor_node = mary_perception.tof_processor_node:main',
        ],
    },
)
