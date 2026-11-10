#!/usr/bin/python3
"""Exercise the real C++ nodes through ROS images, calibration, poses and TF."""
import math
import time
import unittest

import cv2
import numpy as np
from pathlib import Path
import signal
import subprocess
import tempfile
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from aruco_det.msg import MarkerArray
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


def draw_marker(dictionary, marker_id, pixels):
    function = getattr(cv2.aruco, 'generateImageMarker', None)
    if function is None:
        function = cv2.aruco.drawMarker
    return function(dictionary, marker_id, pixels)


class PipelineTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('synthetic_pipeline_test')
        self.processes = []
        self.logs = tempfile.TemporaryDirectory(prefix='aruco_pipeline_')
        self.addCleanup(self.cleanup)
        binary = str(Path(get_package_prefix('aruco_det')) / 'lib/aruco_det/aruco_det_node')
        folder = Path(__file__).resolve().parent
        for ns in ('file', 'info', 'legacy'):
            args = [binary, '--ros-args', '-r', '__ns:=/' + ns,
                    '-p', 'sub_image_topic:=/test/image',
                    '-p', 'camera_info_topic:=/test/camera_info']
            if ns == 'info':
                args += ['-p', 'use_camera_info:=true', '-p', 'publish_tf:=false']
            else:
                name = 'legacy_camera.yaml' if ns == 'legacy' else 'camera.yaml'
                args += ['-p', 'camera_param_path:=' + str(folder / name)]
            if ns == 'legacy':
                args += ['-p', 'target_aruco_ids:=[18]', '-p', 'primary_id:=18',
                         '-p', 'publish_tf:=false']
            log = open(Path(self.logs.name) / (ns + '.log'), 'w+')
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
            self.processes.append((process, log))
        self.messages = {}
        self.counts = {}
        self.subscribers = []
        self.bridge = CvBridge()
        self.publisher = self.node.create_publisher(Image, '/test/image', qos_profile_sensor_data)
        self.info_publisher = self.node.create_publisher(CameraInfo, '/test/camera_info', 10)
        for ns in ('file', 'info', 'legacy'):
            for topic, kind in (('aruco/markers', MarkerArray), ('aruco/pose', PoseStamped),
                                ('aruco/camera_pose', PoseStamped), ('aruco_det_image', Image)):
                self.subscribe('/' + ns + '/' + topic, kind)
        self.subscribe('/tf', TFMessage)
        self.wait(lambda: self.publisher.get_subscription_count() == 3)
        self.wait(lambda: self.info_publisher.get_subscription_count() == 1)
        self.wait(lambda: all(self.node.count_publishers('/' + ns + '/' + suffix) > 0
                  for ns in ('file', 'info', 'legacy')
                  for suffix in ('aruco/markers', 'aruco_det_image')))
        self.spin_for(0.3)

    def read_logs(self):
        result = []
        for process, log in self.processes:
            log.flush()
            log.seek(0)
            result.append(log.read())
        return '\n'.join(result)

    def cleanup(self):
        for process, log in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            log.close()
        self.node.destroy_node()
        rclpy.shutdown()
        self.logs.cleanup()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def subscribe(self, topic, kind):
        def callback(message):
            self.messages[topic] = message
            self.counts[topic] = self.counts.get(topic, 0) + 1
        self.subscribers.append(self.node.create_subscription(kind, topic, callback, 10))

    def wait(self, predicate, timeout=8):
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.fail('Timed out waiting for ROS output; counts: ' + str(self.counts) +
                  '; node logs:\n' + self.read_logs())

    def send(self, image):
        message = self.bridge.cv2_to_imgmsg(image, encoding='mono8')
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'camera_optical_frame'
        self.publisher.publish(message)
        for ns in ('file', 'info', 'legacy'):
            for suffix in ('aruco/markers', 'aruco_det_image'):
                topic = '/' + ns + '/' + suffix
                self.wait(lambda: topic in self.messages and
                          self.messages[topic].header.stamp == message.header.stamp)
        return message.header.stamp

    def camera_info(self, valid=True, frame='camera_optical_frame'):
        info = CameraInfo()
        info.header.frame_id = frame
        info.width, info.height = 640, 480
        info.distortion_model = 'plumb_bob'
        info.k = [float(x) for x in [800 if valid else 0, 0, 320, 0, 800, 240, 0, 0, 1]]
        info.d = [0.0] * 5
        self.info_publisher.publish(info)
        self.spin_for(0.2)

    def test_pipeline(self):
        dictionary = cv2.aruco.getPredefinedDictionary(10)
        scene = np.full((480, 640), 255, dtype=np.uint8)
        # 80 mm black square at 0.8 m with fx=800 projects to 80 pixels.
        scene[200:280, 280:360] = draw_marker(dictionary, 0, 80)
        # ID 18 is left and below the optical axis: X=-0.20 m, Y=+0.06 m.
        scene[260:340, 80:160] = draw_marker(dictionary, 18, 80)

        # Without CameraInfo, still annotate but never fabricate a metric pose.
        self.send(scene)
        self.assertEqual(self.messages['/info/aruco/markers'].markers, [])
        self.camera_info()
        self.subscribe('/file/aruco/id_18/pose', PoseStamped)
        self.spin_for(0.3)
        stamp = self.send(scene)
        for ns in ('file', 'info'):
            markers = self.messages['/' + ns + '/aruco/markers']
            self.assertEqual({m.id for m in markers.markers}, {0, 18})
            self.assertEqual(markers.header.frame_id, 'camera_optical_frame')
            main = next(m for m in markers.markers if m.id == 0)
            self.assertAlmostEqual(main.pose.position.z, 0.8, delta=0.02)
            self.assertAlmostEqual(main.pose.position.x, 0, delta=0.005)
            self.assertAlmostEqual(main.pose.position.y, 0, delta=0.005)
            off_centre = next(m for m in markers.markers if m.id == 18)
            self.assertAlmostEqual(off_centre.pose.position.x, -0.2, delta=0.005)
            self.assertAlmostEqual(off_centre.pose.position.y, 0.06, delta=0.005)
            self.assertAlmostEqual(off_centre.pose.position.z, 0.8, delta=0.02)
            q = main.pose.orientation
            self.assertTrue(all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)))
            self.assertAlmostEqual(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1, places=8)
            self.assertGreater(abs(q.x), 0.99)  # Regression: near-pi rotation formerly divided by w=0.
            self.assertLess(main.reprojection_error, 1.0)
            pose_topic = '/' + ns + '/aruco/pose'
            inverse_topic = '/' + ns + '/aruco/camera_pose'
            self.wait(lambda: self.messages.get(pose_topic) is not None and
                      self.messages[pose_topic].header.stamp == stamp)
            self.wait(lambda: self.messages.get(inverse_topic) is not None and
                      self.messages[inverse_topic].header.stamp == stamp)
            self.assertEqual(self.messages[pose_topic].pose, main.pose)
            inverse = self.messages[inverse_topic]
            self.assertEqual(inverse.header.frame_id, 'aruco_0')
            self.assertAlmostEqual(inverse.pose.position.z, 0.8, delta=0.02)
            self.assertAlmostEqual(inverse.pose.position.x, 0, delta=0.005)
            self.assertAlmostEqual(inverse.pose.position.y, 0, delta=0.005)
        self.assertEqual([m.id for m in self.messages['/legacy/aruco/markers'].markers], [18])
        self.wait(lambda: self.messages.get('/file/aruco/id_18/pose') is not None and
                  self.messages['/file/aruco/id_18/pose'].header.stamp == stamp)
        for topic in ('/file/aruco/id_18/pose', '/legacy/aruco/pose'):
            self.wait(lambda: self.messages.get(topic) is not None and
                      self.messages[topic].header.stamp == stamp)
            position = self.messages[topic].pose.position
            self.assertAlmostEqual(position.x, -0.2, delta=0.005)
            self.assertAlmostEqual(position.y, 0.06, delta=0.005)
            self.assertAlmostEqual(position.z, 0.8, delta=0.02)
        self.wait(lambda: '/tf' in self.messages and any(
            t.header.stamp == stamp and t.child_frame_id in ('aruco_0', 'aruco_18')
            and t.header.frame_id == 'camera_optical_frame' for t in self.messages['/tf'].transforms))

        # Stop image publication: the detector must not reprocess or republish the old frame.
        self.spin_for(0.2)
        counts = dict(self.counts)
        self.spin_for(0.5)
        self.assertEqual(self.counts, counts)

        # Disappearance produces an empty array and no stale pose.
        old_stamp = self.messages['/file/aruco/pose'].header.stamp
        self.send(np.full_like(scene, 255))
        for ns in ('file', 'info', 'legacy'):
            self.assertEqual(self.messages['/' + ns + '/aruco/markers'].markers, [])
        self.assertEqual(self.messages['/file/aruco/pose'].header.stamp, old_stamp)

        # Resolution mismatch must suppress metric output in both calibration paths.
        self.send(cv2.resize(scene, (320, 240)))
        for ns in ('file', 'info', 'legacy'):
            self.assertEqual(self.messages['/' + ns + '/aruco/markers'].markers, [])

        # A new invalid calibration must invalidate the previous valid calibration.
        self.camera_info(valid=False)
        self.send(scene)
        self.assertEqual(self.messages['/info/aruco/markers'].markers, [])
        self.assertEqual(len(self.messages['/file/aruco/markers'].markers), 2)
        self.camera_info(frame='wrong_camera_frame')
        self.send(scene)
        self.assertEqual(self.messages['/info/aruco/markers'].markers, [])
        self.camera_info()
        self.send(scene)
        self.assertEqual(len(self.messages['/info/aruco/markers'].markers), 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
