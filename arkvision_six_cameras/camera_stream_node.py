import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')
        self.declare_parameter('num_cameras', 6)
        self.declare_parameter('publish_rate', 20)

        num_cameras = self.get_parameter('num_cameras').value
        publish_rate = self.get_parameter('publish_rate').value

        self.bridge = CvBridge()
        self.publishers = []
        self.timers = []

        base_ip = 70
        for i in range(num_cameras):
            camera_ip = f"rtsp://192.168.26.{base_ip+i}:8554/jpeg"
            publisher = self.create_publisher(Image, f'camera/{i}/image', 10)
            timer = self.create_timer(1.0 / publish_rate, self.create_camera_callback(camera_ip, publisher))
            self.publishers.append(publisher)
            self.timers.append(timer)

    def create_camera_callback(self, camera_ip, publisher):
        def camera_callback():
            cap = cv2.VideoCapture(camera_ip)
            ret, frame = cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                publisher.publish(msg)
            cap.release()
        return camera_callback


def main(args=None):
    rclpy.init(args=args)
    camera_stream_node = CameraStreamNode()
    rclpy.spin(camera_stream_node)
    camera_stream_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




'''import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import signal
import ipaddress

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')
        self.declare_parameter('base_camera_ip', '192.168.26.70')
        base_camera_ip = self.get_parameter('base_camera_ip').value

		# Generate the next five IP addresses
		self.camera_urls = generate_rtsp_urls(base_camera_ip)
		
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

def generate_rtsp_urls(ip_address):
    # Split the IP address into its components
    parts = ip_address.split('.')
    if len(parts) != 4:
        raise ValueError("Invalid IP address format")

    # Convert the last part to an integer for manipulation
    last_octet = int(parts[-1])
    base_octets = parts[:-1]  # Get the first three octets as the base

    # Generate the next five IP addresses
    ip_addresses = []
    for i in range(6):  # Including the original and the next five
        new_last_octet = last_octet + i
        # Ensure the new_last_octet is not greater than 255
        if new_last_octet > 255:
            raise ValueError("IP address exceeds the valid range")
        new_ip = '.'.join(base_octets + [str(new_last_octet)])
        ip_addresses.append(new_ip)

    # Format the IP addresses with the specified format
    rtsp_urls = [f"rtsp://{ip}:8554/h264" for ip in ip_addresses]

    return rtsp_urls


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
    main()'''

