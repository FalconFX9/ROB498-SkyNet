import os
from glob import glob
from setuptools import setup

package_name = 'mary_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@mail.utoronto.ca',
    description='Hardware interface package for MARY drone - MAVROS config, sensor drivers',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
