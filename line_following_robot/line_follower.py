import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

class LineFollowingNode(Node):
    def __init__(self):
        # Initializes the ROS2 node
        super().__init__("line_follower")

        # Subscribes to the camera image topic
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.callback_function, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Creates bridge to transfer image data between ROS and OpenCV
        self.cv_bridge = CvBridge()

        # Contains the largest contour encountered
        self.largestContour = 0

    def callback_function(self, data):
        # Converts ROS image message to OpenCV image
        frame = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Creates gray scale of the cropped camera view
        cropped_frame = frame[310:480, 0:640]
        cropped_frame_gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)

        # Converts pixels with values above 100 to black and values 100 or below to white
        _, thresh = cv2.threshold(cropped_frame_gray, 100, 255, cv2.THRESH_BINARY_INV)

        # Stores a list of contours from the thresholded image
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Selects the the contour with the largest area which is assumed to be the black line
        if len(contours) != 0:
            self.largestContour = contours[0]
            for contour in contours:
                if cv2.contourArea(contour) > cv2.contourArea(self.largestContour):
                    self.largestContour = contour

        # Creates a bounding rect that represents the detected line's position, width, and height
        x, y, width, height = cv2.boundingRect(self.largestContour)
        cv2.rectangle(cropped_frame, (x, y), (x + width, y + height), (0, 255, 0), 2)

        # Calculates the horizontal center of the rectangle
        center = (x + (x + width)) / 2
        
        # Adjusts the vehicle's speed and direction to align the center of the camera's view
        # with the center of the black line
        twist = Twist()
        if center < 319.5:
            twist.angular.z = 5.5 * (320 - center) / 320
            twist.linear.x = 0.2
            self.cmd_vel_publisher.publish(twist)
        elif center > 320.5:
            twist.angular.z = -5.5 * (center - 320) / 320
            twist.linear.x = 0.2
            self.cmd_vel_publisher.publish(twist)
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.4
            self.cmd_vel_publisher.publish(twist)

        # Displays camera feed on a window
        cv2.imshow("Camera Cropped View", cropped_frame)
        cv2.imshow("Camera Full View", frame)
        cv2.waitKey(1)



def main(args=None):
    # ROS2 lifecycle management
    rclpy.init(args=args)
    node = LineFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
