import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.image_count = 0
        self.image_sub = self.create_subscription(
            Image,
            '/camera_1/image_raw',  # Replace with your image topic
            self.image_callback,
            10
        )

        # Create a directory to store images
        if not os.path.exists('extracted_images'):
            os.makedirs('extracted_images')

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Save the image to a file
            img_filename = f'extracted_images/image_{self.image_count}.jpg'
            cv2.imwrite(img_filename, cv_image)
            self.get_logger().info(f"Saved image {img_filename}")

            self.image_count += 1
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    
    # Spin to keep the node running
    rclpy.spin(image_saver)
    
    # Shutdown after spinning
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
