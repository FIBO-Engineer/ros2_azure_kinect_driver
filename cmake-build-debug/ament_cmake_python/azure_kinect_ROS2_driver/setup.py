from setuptools import find_packages
from setuptools import setup

setup(
    name='azure_kinect_ROS2_driver',
    version='0.0.1',
    packages=find_packages(
        include=('azure_kinect_ROS2_driver', 'azure_kinect_ROS2_driver.*')),
)
