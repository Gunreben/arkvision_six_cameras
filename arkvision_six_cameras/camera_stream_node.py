import cv2 ###Needs to be in first place, no idea, why.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import threading
from rclpy.executors import MultiThreadedExecutor

class CameraStreamNode(Node):
    """
    Node for streaming video from a camera over ROS2.
    """
    def __init__(self, camera_ip='192.168.26.70'):
        """
        Initialize the camera stream node.
        
        :param camera_ip: IP address of the camera
        """
        camera_label = self._get_camera_label(camera_ip)
        node_name = f'camera_stream_node_{camera_label}'
        super().__init__(node_name)

        self.declare_parameter('publish_rate', 15.0)

        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._logger.info('Creating Camera Publisher...')
        self.publisher_ = self.create_publisher(Image, f'camera_image/cam_{camera_label}', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / publish_rate, self._timer_callback)

        #TODO: optimize this!
        """
        gst_pipeline = (
            f"rtspsrc location=rtsp://{camera_ip}:8554/jpeg latency=80 ! "
            "rtpjpegdepay ! "
            "nvv4l2decoder mjpeg=1 ! "
            "nvvidconv ! "
            "video/x-raw,format=BGRx ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink"
        )

        # Create a VideoCapture object with the GStreamer pipeline
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        ###working but with huge delay
        #self.cap = cv2.VideoCapture(f"rtsp://{camera_ip}:8554/h264")
        """
        # Initializing video capture with HTTP protocol
        self.cap = cv2.VideoCapture(f"http://{camera_ip}:81")
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera stream.')
            sys.exit(1)

    def _timer_callback(self):
        """
        Timer callback to publish camera frames.
        """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame from camera.')
            return

        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        self.publisher_.publish(ros_image)

    def _get_camera_label(self, camera_ip):
        """
        Get the camera label based on its IP address.
        
        :param camera_ip: IP address of the camera
        :return: Corresponding camera label
        """
        last_part = camera_ip.split('.')[-1]
        ip_dictionary = {'70': 'FL', '71': 'ML', '72': 'BL', '73': 'BR', '74': 'MR', '75': 'FR'}
        return ip_dictionary.get(last_part, last_part)

def main(args=None):
    """
    Main function to run the camera stream nodes.
    """
    rclpy.init(args=args)
    camera_ips = ['192.168.26.70', '192.168.26.71', '192.168.26.72', '192.168.26.73', '192.168.26.74', '192.168.26.75']
    nodes = []
    executor = MultiThreadedExecutor(num_threads=6)

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
