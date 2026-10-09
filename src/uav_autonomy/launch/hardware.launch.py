from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='42'),

        # ArduPilot DDS bridge: serial to the Pixhawk (Telem2).
        # -b must match SERIAL2_BAUD on the FC exactly (2000 -> 2000000, 921 -> 921600).
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
            output='screen',
        ),

        # Arducam OV9782 (USB UVC global shutter) via usb_cam.
        # Prefer a stable /dev/v4l/by-id/... path if the video index changes after reboot.
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'pixel_format': 'mjpeg2rgb',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'frame_id': 'camera_optical_frame',
                # 'camera_info_url': 'file:///home/ubuntu/.ros/camera_info/ov9782.yaml',
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ],
        ),

        # base_link -> camera_link (mechanical mount offset only).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_mount_tf',
            output='screen',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '-0.06',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
            ],
        ),

        # camera_link -> camera_optical_frame (camera looks down:
        # optical Z = down, X = drone right, Y = backward).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '3.14159265', '--pitch', '0', '--yaw', '-1.57079633',
                '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame',
            ],
        ),

        Node(
            package='uav_vision',
            executable='aruco_detector_node',
            name='aruco_detector',
            output='screen',
        ),

        Node(
            package='uav_autonomy',
            executable='landing_manager_node',
            name='landing_manager',
            output='screen',
            parameters=[{
                'target_aruco_id': 0,
                'landing_altitude_threshold': 0.0,
            }],
        ),
    ])
