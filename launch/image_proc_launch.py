from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


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


    ld.add_action(ip_rgb_node)
    ld.add_action(ip_depth_node)
    return ld
