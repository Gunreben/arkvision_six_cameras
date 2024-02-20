import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import signal

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')
        self.declare_parameter('base_camera_ip', '192.168.26.70')
        base_camera_ip = self.get_parameter('base_camera_ip').value
        self.camera_urls = [f"rtsp://{base_camera_ip[:-1]}{i}:8554/h264" for i in range(int(base_camera_ip.split('.')[-1]), int(base_camera_ip.split('.')[-1]) + 6)]
        self.cv_bridge = CvBridge()
        self.shutdown_flag = threading.Event()  # Shutdown flag
        for url in self.camera_urls:
            threading.Thread(target=self.capture_and_publish, args=(url,), daemon=True).start()

    def capture_and_publish(self, camera_url):
        cap = cv2.VideoCapture(camera_url)
        publisher = self.create_publisher(Image, f'camera/image_{camera_url[-2:]}', 10)
        while not self.shutdown_flag.is_set() and rclpy.ok():  # Check shutdown flag
            ret, frame = cap.read()
            if ret:
                msg = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
                publisher.publish(msg)
            else:
                self.get_logger().warn(f'Camera stream {camera_url} not available')
        cap.release()  # Release the VideoCapture when done

def main(args=None):
    rclpy.init(args=args)
    camera_stream_node = CameraStreamNode()

    def signal_handler(sig, frame):  # Define signal handler
        camera_stream_node.shutdown_flag.set()  # Set shutdown flag
        camera_stream_node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)  # Register signal handler

    rclpy.spin(camera_stream_node)

if __name__ == '__main__':
    main()

