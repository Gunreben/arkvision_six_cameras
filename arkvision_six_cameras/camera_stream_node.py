import cv2  # Needs to be in first place; otherwise, some systems may have issues
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import sys
import threading

class CameraStreamNode(Node):
    """
    Node for streaming video from a camera over ROS2,
    allowing both raw and compressed image transport based on a parameter.
    """
    def __init__(self, camera_ip='192.168.26.70'):
        """
        Initialize the camera stream node.
        
        :param camera_ip: IP address of the camera
        """
        camera_label = self._get_camera_label(camera_ip)
        node_name = f'camera_stream_node_{camera_label}'
        super().__init__(node_name)

        # Declare and get parameters
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('use_compressed', True)
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value

        # Prepare logging
        self._logger = self.get_logger()
        self._logger.info('Creating Camera Publisher...')

        # Bridge for raw image conversion
        self.bridge = CvBridge()

        # Create publisher (raw or compressed)
        if use_compressed:
            # Use CompressedImage messages
            self._logger.info(f"Publishing compressed images for camera {camera_label}")
            self.publisher_ = self.create_publisher(
                CompressedImage,
                f'camera_image/Cam_{camera_label}',
                10
            )
        else:
            # Use raw sensor_msgs/Image messages
            self._logger.info(f"Publishing raw images for camera {camera_label}")
            self.publisher_ = self.create_publisher(
                Image,
                f'camera_image/Cam_{camera_label}',
                10
            )

        # Create a timer callback for publishing images
        self.timer = self.create_timer(1.0 / publish_rate, self._timer_callback)

        # Initialize video capture
        # TODO: Maybe customize as needed (e.g. using GStreamer, RTSP, HTTP, etc.)
        self.cap = cv2.VideoCapture(f"http://{camera_ip}:81")
        
        if not self.cap.isOpened():
            self._logger.error('Failed to open camera stream.')
            sys.exit(1)

        # Save whether we are using compressed
        self.use_compressed = use_compressed

    def _timer_callback(self):
        """
        Timer callback to capture frames and publish as raw or compressed.
        """
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self._logger.error('Failed to capture frame from camera.')
            return

        if self.use_compressed:
            # Publish as CompressedImage
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'  # or 'png', etc.

            # Perform JPEG compression
            success, encoded_image = cv2.imencode('.jpg', frame)
            if not success:
                self._logger.error('Failed to encode frame as JPEG.')
                return

            msg.data = encoded_image.tobytes()
            self.publisher_.publish(msg)
        else:
            # Publish as raw Image using CvBridge
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(ros_image)

    def _get_camera_label(self, camera_ip):
        """
        Get the camera label based on its IP address.
        """
        last_part = camera_ip.split('.')[-1]
        ip_dictionary = {
            '70': 'FL',
            '71': 'ML',
            '72': 'BL',
            '73': 'BR',
            '74': 'MR',
            '75': 'FR'
        }
        return ip_dictionary.get(last_part, last_part)

def main(args=None):
    """
    Main function to run the camera stream nodes.
    """
    rclpy.init(args=args)
    camera_ips = ['192.168.26.70', '192.168.26.71', '192.168.26.72', 
                  '192.168.26.73', '192.168.26.74', '192.168.26.75']

    executor = MultiThreadedExecutor(num_threads=len(camera_ips))
    nodes = []

    for ip in camera_ips:
        node = CameraStreamNode(ip)
        executor.add_node(node)
        nodes.append(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    print("Executor started!")

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
