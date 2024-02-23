import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import threading
from rclpy.executors import MultiThreadedExecutor

class CameraStreamNode(Node):
    def __init__(self, camera_ip = '192.168.26.70'):
        
        camera_label = self.get_camera_label(camera_ip)

        super().__init__('camera_stream_node_num' + camera_label)

        self.declare_parameter('publish_rate', 50.0)
        #self.declare_parameter('camera_ip', '192.168.26.70')  # Declare camera_ip parameter with default value

        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        #camera_ip = self.get_parameter('camera_ip').get_parameter_value().string_value  # Get the camera_ip paramete

        self.get_logger().info('Creating Camera Publisher...')



        self.publisher_ = self.create_publisher(Image, 'camera_image/cam_' + camera_label, 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        ### currently not working at all.
        #self.cap = cv2.VideoCapture(f"rtsp://{camera_ip}:8554/jpeg")
        ###working but with huge delay
        #self.cap = cv2.VideoCapture(f"rtsp://{camera_ip}:8554/h264")

        self.cap = cv2.VideoCapture(f"http://{camera_ip}:81")  # Use the camera_ip parameter

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera stream.')
            sys.exit(1)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame from camera.')
            return

        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough") ###problematic?
        self.publisher_.publish(ros_image)

    def get_camera_label(self, camera_ip):
        parts = camera_ip.split('.')
        # Get the last part of the IP address
        last_part = parts[-1]
        #dictionary with camlabels
        ip_dictionary = {'70': 'FL','71': 'FR','72': 'BL','73': 'BR','74': 'FC','75': 'BC'}        
        # Get the corresponding label for the last part of the IP address
        label = ip_dictionary.get(last_part, last_part)
        return label

def main(args=None):
    rclpy.init(args=args)

    #TODO: Parse this as parameter
    camera_ips = ['192.168.26.70', '192.168.26.71', '192.168.26.72', '192.168.26.73', '192.168.26.74', '192.168.26.75']
    
    nodes = []  # List to keep track of created nodes

    # Create an executor for managing threads
    executor = MultiThreadedExecutor(num_threads=6)


    # Create and add nodes for each camera IP to the executor
    for ip in camera_ips:
        node = CameraStreamNode(ip)
        executor.add_node(node)
        nodes.append(node)  # Store the node for potential future use

    # Use a separate thread for the executor to allow for parallel callback processing
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    print("executor started!")

    try:
        # Keep the main thread alive until a KeyboardInterrupt (Ctrl+C) is received.
        executor_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup: Stop the executor and shutdown rclpy
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
