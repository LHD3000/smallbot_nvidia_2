"""USB camera and ArUco pipeline for ROS 2 Humble and Jazzy."""
import math
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def boolean(value):
    if value.lower() in ('true', '1'):
        return True
    if value.lower() in ('false', '0'):
        return False
    raise ValueError('Expected true/false, got: ' + value)


def launch_nodes(context, defaults):
    value = lambda key: LaunchConfiguration(key).perform(context)
    namespace = value('namespace')
    result = []

    def required(node):
        result.extend([node, RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[EmitEvent(event=Shutdown(
                reason='Required camera/detector process exited'))]))])

    if boolean(value('start_camera')):
        pixel_format = value('pixel_format')
        # Preserve the old CLI while using the ROS 2 usb_cam conversion names.
        pixel_format = {'yuyv': 'yuyv2rgb', 'mjpeg': 'mjpeg2rgb'}.get(pixel_format, pixel_format)
        url = value('camera_info_url')
        if not url and value('camera_param_path'):
            url = Path(value('camera_param_path')).expanduser().resolve().as_uri()
        if not boolean(value('load_camera_info')):
            url = ''
        fps = float(value('framerate'))
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError('framerate must be finite and positive')
        required(Node(
            package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
            namespace=namespace, output='screen', parameters=[{
                # Older usb_cam releases resolve relative udev symlinks incorrectly.
                'video_device': str(Path(value('video_device')).resolve()),
                'image_width': int(value('image_width')),
                'image_height': int(value('image_height')), 'framerate': fps,
                'pixel_format': pixel_format, 'io_method': 'mmap',
                'frame_id': value('camera_frame_id'), 'camera_name': 'usb_camera',
                'camera_info_url': url, 'brightness': -1,
            }], remappings=[('image_raw', value('sub_image_topic')),
                            ('camera_info', value('camera_info_topic'))]))
    if boolean(value('start_detector')):
        parameters = {}
        for key, default in defaults.items():
            converter = boolean if isinstance(default, bool) else type(default)
            parameters[key] = converter(value(key))
        parameters['camera_param_path'] = value('camera_param_path')
        reliability = value('image_qos_reliability')
        if reliability == 'auto':
            reliability = 'reliable' if boolean(value('start_camera')) else 'best_effort'
        parameters['image_qos_reliability'] = reliability
        ids = yaml.safe_load(value('target_aruco_ids'))
        if not isinstance(ids, list) or any(type(item) is not int for item in ids):
            raise ValueError('target_aruco_ids must be an integer list, e.g. [18,19]')
        # Omit [] so rclcpp retains its declared INTEGER_ARRAY parameter type.
        if ids:
            parameters['target_aruco_ids'] = ids
        required(Node(package='aruco_det', executable='aruco_det_node', name='aruco_det',
                      namespace=namespace, output='screen', parameters=[parameters]))
    if boolean(value('view')):
        if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
            raise RuntimeError('view:=true requires a desktop display')
        result.append(Node(package='aruco_det', executable='view_image.py', name='aruco_view',
                           namespace=namespace, output='screen',
                           parameters=[{'topic': value('view_topic')}]))
    return result


def generate_launch_description():
    share = Path(get_package_share_directory('aruco_det'))
    defaults = yaml.safe_load((share / 'config/aruco.yaml').read_text())['aruco_det']['ros__parameters']
    extra = {
        'video_device': '/dev/video0', 'image_width': '640', 'image_height': '480',
        'framerate': '30.0', 'pixel_format': 'yuyv', 'load_camera_info': 'true',
        'camera_info_url': '', 'camera_param_path': str(share / 'config/usb_camera.yaml'),
        'target_aruco_ids': '[]', 'start_camera': 'true', 'start_detector': 'true',
        'view': 'false', 'view_topic': '/aruco_det_image', 'namespace': '',
        'image_qos_reliability': 'auto',
    }
    arguments = [DeclareLaunchArgument(key, default_value=str(default).lower()
                 if isinstance(default, bool) else str(default))
                 for key, default in {**defaults, **extra}.items()]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_nodes, kwargs={'defaults': defaults})])
