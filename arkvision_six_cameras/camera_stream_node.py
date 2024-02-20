import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')
        # Declare the base camera IP parameter
        self.declare_parameter('base_camera_ip', '192.168.26.70')
        base_camera_ip = self.get_parameter('base_camera_ip').value

        # Extract the base for the IP and the starting number for the last octet
        ip_base = ".".join(base_camera_ip.split('.')[:-1])
        start_number = int(base_camera_ip.split('.')[-1])

        # Generate camera URLs based on the base IP and incrementing the last octet
        self.camera_urls = [f"rtsp://{ip_base}.{i}:8554/h264" for i in range(start_number, start_number + 6)]

        self.cv_bridge = CvBridge()

        # For each camera URL, start a separate thread to capture and publish frames
        for url in self.camera_urls:
            threading.Thread(target=self.capture_and_publish, args=(url,), daemon=True).start()

            
    def capture_and_publish(self, camera_url):
        cap = cv2.VideoCapture(camera_url)
        publisher = self.create_publisher(Image, f'camera/image_{camera_url[-2:]}', 10)

        while rclpy.ok():
            ret, frame = cap.read()
            if ret:
                msg = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
                publisher.publish(msg)
            else:
                self.get_logger().warn(f'Camera stream {camera_url} not available')

def main(args=None):
    rclpy.init(args=args)
    camera_stream_node = CameraStreamNode()
    rclpy.spin(camera_stream_node)
    camera_stream_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

