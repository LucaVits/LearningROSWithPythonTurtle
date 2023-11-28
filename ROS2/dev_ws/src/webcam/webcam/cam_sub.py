import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from cv_bridge.core import CvBridgeError
import numpy as np
import sys

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    
    return image_opencv

def cv2_to_imgmsg(cv_image):
    
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

class WebcamSub(Node):
    def __init__(self):
        super().__init__('stream_node')

        self.bridge = CvBridge()

        # define subscriber
        self.img_subscription = self.create_subscription(Image, 'image_raw', self.img_callback, 1)
        self.img_subscription # prevent unused varaibale warning

    def img_callback(self, img_msg):
        
        # bridging from img msg to cv2 img
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            # If you're using win10, uncomment the line below and comment the line above
            # cv_image = imgmsg_to_cv2(img_msg)
        except CvBridgeError as e:
            self.get_logger().info(e)
        
        # show image
        cv2.namedWindow("Image")
        if cv_image is not None:
            cv2.imshow("Image", cv_image)
        cv2.waitKey(1)

def main(args=None):

    rclpy.init(args=args)

    imgsub_obj = WebcamSub()
    rclpy.spin(imgsub_obj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imgsub_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()