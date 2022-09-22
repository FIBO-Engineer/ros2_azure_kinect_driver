from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    k4a_node = Node(
        package="azure_kinect_ros2_driver",
        executable="azure_kinect_node",
        name="k4a_ros2_node",
        output="screen",
        emulate_tty=True
    )

    ip_rgb_node = Node(
        package="image_proc",
        executable="image_proc",
        remappings=[
            ('image', '/k4a/rgb_to_depth/image_raw'),
            ('image_rect', '/k4a/rgb_to_depth/image_rect'),
            ('camera_info', '/k4a/rgb_to_depth/camera_info'),
        ]
    )

    ip_depth_node = Node(
        package="image_proc",
        executable="image_proc",
        remappings=[
            ('image', '/k4a/depth/image_raw'),
            ('image_rect', '/k4a/depth/image_rect'),
            ('camera_info', '/k4a/depth/camera_info'),
        ]
    )



    ld.add_action(k4a_node)
    ld.add_action(ip_rgb_node)
    ld.add_action(ip_depth_node)
    return ld
