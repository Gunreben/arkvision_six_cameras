import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')
        self.declare_parameter('publish_rate', 60.0)
        self.declare_parameter('camera_ip', '192.168.26.70')  # Declare camera_ip parameter with default value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        camera_ip = self.get_parameter('camera_ip').get_parameter_value().string_value  # Get the camera_ip parameter

        self.publisher_ = self.create_publisher(Image, 'camera_image/{camera_ip}', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.cap = cv2.VideoCapture(f"rtsp://{camera_ip}:8554/h264")  # Use the camera_ip parameter

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera stream.')
            sys.exit(1)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame from camera.')
            return

        ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher_.publish(ros_image)
        #self.get_logger().info('Publishing camera frame.')

def main(args=None):
    rclpy.init(args=args)
    camera_stream_node = CameraStreamNode()
    rclpy.spin(camera_stream_node)
    camera_stream_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

