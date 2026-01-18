import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class LineFollowingNode(Node):
    def __init__(self):
        # Initializes the ROS2 node
        super().__init__("line_follower")

        # Subscribes to the camera image topic
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.callback_function, 10)
        self.cv_bridge = CvBridge()

    def callback_function(self, data):
        # Converts ROS image message to OpenCV image
        frame = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Displays camera feed on a window
        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)

def main(args=None):
    # ROS2 lifecycle management
    rclpy.init(args=args)
    node = LineFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
