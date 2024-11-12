import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class RightLeftNode(Node):
    def __init__(self):
        super().__init__('right_left_node')
        
        self.x_position = 0.0 # Set this in the pose_callback
        self.going_right = False # Use this for your velocity logic
        
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # Edit this line: What topic do you want to publish to?
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.create_timer(1, self.publish_velocity)
        
    def pose_callback(self, msg: Pose):
        # The type of `msg` is a turtlesim.Pose. To get the x position of pose, do:
        # `msg.x`
        self.x_position = msg.x
        
    
    def publish_velocity(self):
        # cmdvel accepts a Twist: A message with both a linear and angular component.
        # For this activity, we only care about linear motion in the x axis.
        # You can set that like: `msg.linear.x = 1`
        msg = Twist()
        
        # Put your logic here. How can you access the current position of the turtle?
        # Switch to going right (bigger x) when x is less than 1, and switch to going
        # left (smaller x) when x is greater than 10
        if self.x_position < 1:
            self.going_right = True
        elif (self.x_position > 10):
            self.going_right = False

        if self.going_right:
            msg.linear.x = 2.0
        else:
            msg.linear.x = -2.0
        
        self.velocity_publisher.publish(msg)
        
    

def main(args=None):
    print('Starting right-left node...')
    rclpy.init(args=args)
    publisher = RightLeftNode()
    rclpy.spin(publisher)


if __name__ == '__main__':
    main()
