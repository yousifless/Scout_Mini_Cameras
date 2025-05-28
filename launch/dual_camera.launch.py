import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_1',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 10.0,
                'pixel_format': 'uyvy',
                'image_width': 2880,
                'image_height': 1860,
                'frame_id': 'camera'
            }],
            remappings=[('/image_raw', '/camera_1/image_raw')]  # Remapping /image_raw to /camera_1/image_raw
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_2',
            output='screen',
            parameters=[{
                'video_device': '/dev/video4',
                'framerate': 10.0,
                'pixel_format': 'uyvy',
                'image_width': 2880,
                'image_height': 1860,
                'frame_id': 'camera'
            }],
            remappings=[('/image_raw', '/camera_2/image_raw')]  # Remapping /image_raw to /camera_2/image_raw
        ),
    ])

