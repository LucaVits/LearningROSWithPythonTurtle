import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

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

class Webcam_Impl(Node): 
    def __init__(self):
        super().__init__('webcam')

        # initialize a publisher
        self.img_publisher = self.create_publisher(Image, 'image_raw', 1)

        #initializa camera parameters
        self.camera = cv2.VideoCapture(0)
        self.camera.set(3,320)
        self.camera.set(4,240)

        self.bridge=CvBridge()

        # create timer
        self.timer = self.create_timer(0.03, self.capture_frame)

    def capture_frame(self):

        rval,img_data = self.camera.read()
        if rval:
            self.img_publisher.publish(self.bridge.cv2_to_imgmsg(img_data, "bgr8"))
            # If you're using win10, uncomment the line below and comment the line above
            # self.img_publisher.publish(cv2_to_imgmsg(img_data))
            return img_data 
        else:
            print("error")

def main(args=None):

    # initial a ros2
    rclpy.init(args=args)

    # initialize a camera object/ros2 node
    webcam=Webcam_Impl()
    webcam.get_logger().info('Webcam Node Started!') 

    # spin the node (otherwise it only execute once)
    rclpy.spin(webcam)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    webcam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    