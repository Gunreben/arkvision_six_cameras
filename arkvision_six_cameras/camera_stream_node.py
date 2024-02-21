import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')
        self.bridge = CvBridge()
        self.cameras = []
        self.publishers = []
        self.declare_parameter('publish_rate', 60.0)
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        for i in range(70, 76):  # For cameras with IPs from 192.168.26.70 to 192.168.26.75
            camera_ip = f"rtsp://192.168.26.{i}:8554/h264"
            cap = cv2.VideoCapture(camera_ip)
            if not cap.isOpened():
                self.get_logger().error(f'Failed to open camera stream for IP: {camera_ip}')
                continue
            self.cameras.append(cap)
            publisher = self.create_publisher(Image, f'camera_image_{i}', 10)
            self.publishers.append(publisher)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def timer_callback(self):
        for i, cap in enumerate(self.cameras):
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error(f'Failed to capture frame from camera {i+70}')
                continue
            ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publishers[i].publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    camera_stream_node = CameraStreamNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(camera_stream_node, executor=executor)

    camera_stream_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
