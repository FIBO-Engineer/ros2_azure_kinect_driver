import launch
from launch_ros.actions import Node

def generate_launch_description():
    ld = launch.LaunchDescription()

    k4a_node = Node(
        package="azure_kinect_ros2_driver",
        executable="azure_kinect_node",
        name="k4a_ros2_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"recording_file": "~/data_capture/k4a/test1_1536p_nfov.mkv"}
        ]
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

    k4a_test_node = Node(
        package="azure_kinect_ros2_driver",
        executable="test_azure_kinect_subscriber.py",
        output="screen",
        emulate_tty=True
    )

    req_exit  = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=k4a_node,
            on_exit=[
                launch.actions.LogInfo(
                    msg="azure_kinect_node finished; shutting down all nodes"),
                launch.actions.EmitEvent(
                    event=launch.events.Shutdown())]))


    ld.add_action(k4a_node)
    ld.add_action(ip_rgb_node)
    ld.add_action(ip_depth_node)
    ld.add_action(k4a_test_node)
    ld.add_action(req_exit)

    return ld
