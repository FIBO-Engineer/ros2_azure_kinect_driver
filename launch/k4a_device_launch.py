from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    k4a_node = Node(
        package="ros2_azure_kinect_driver",
        executable="azure_kinect_node",
        name="k4a_ros2_node",
        output="screen",
        emulate_tty=True
    )



    ld.add_action(k4a_node)
    return ld
