import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import sys

"""
gunreben@optimus:~/ros2_ws$ ros2 run arkvision_six_cameras camera_stream_node
Exception in thread Thread-1 (<lambda>):
Traceback (most recent call last):
  File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
    self.run()
  File "/usr/lib/python3.10/threading.py", line 953, in run
    self._target(*self._args, **self._kwargs)
  File "/home/gunreben/ros2_ws/install/arkvision_six_cameras/lib/python3.10/site-packages/arkvision_six_cameras/camera_stream_node.py", line 47, in <lambda>
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 222, in spin
    executor.spin_once()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 739, in spin_once
    self._spin_once_impl(timeout_sec)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 728, in _spin_once_impl
    handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 711, in wait_for_ready_callbacks
    return next(self._cb_iter)
ValueError: generator already executing
"""

class CameraStreamNode(Node):
    def __init__(self, camera_id, camera_ip):
        super().__init__('camera_stream_node_{}'.format(camera_id))
        self.camera_id = camera_id
        self.camera_ip = camera_ip
        self.declare_parameter('publish_rate', 60.0)
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Image, 'camera_image_{}'.format(camera_id), 10)
        self.bridge = CvBridge()
        self.timer = None
        self.cap = cv2.VideoCapture("rtsp://{}:8554/h264".format(camera_ip))

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera stream for IP: {}'.format(camera_ip))
            sys.exit(1)

    def start_capturing(self):
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame from camera {}'.format(self.camera_id))
            return

        ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher_.publish(ros_image)
        #self.get_logger().info('Publishing camera frame from {}'.format(self.camera_id))

def main(args=None):
    rclpy.init(args=args)
    nodes = []

    for i in range(70, 76):  # For cameras with IPs from 192.168.26.70 to 192.168.26.75
        camera_ip = "192.168.26.{}".format(i)
        node = CameraStreamNode(i - 69, camera_ip)
        nodes.append(node)
        threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
        node.start_capturing()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        for node in nodes:
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

