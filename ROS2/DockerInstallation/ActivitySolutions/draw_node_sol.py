import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from turtlesim.srv import TeleportAbsolute
import math
import colorsys

star = [(5.0, 1.0), (7.0, 8.0), (1.0, 7.0), (9.0, 7.0), (3.0, 8.0)]

zigzag = [(1.0, 1.0), (2.0, 2.0), (3.0, 1.0), (4.0, 2.0), (5.0, 1.0)]

goals = zigzag

class DrawNode(Node):
    def __init__(self):
        super().__init__('draw_node')        
        self.next_goal = 0
        
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        # TODO I can use rotate_absolute as an intro to clients, to make a more accurate path.
        
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.teleport(goals[0]) # TP to starting point
        
    def teleport(self, position):
        req = TeleportAbsolute.Request()
        req.x = position[0]
        req.y = position[1]
        self.teleport_client.call_async(req)
        # We don't care what the result is, so don't return it
    
    def set_pen(self, r, g, b):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = 2
        self.set_pen_client.call_async(req)
        # We don't care what the result is, so don't return it
                
    def pose_callback(self, pos: Pose):
        vel = Twist()
        if self.close_to_pos(pos, goals[self.next_goal]):
            self.next_goal = (self.next_goal + 1) % len(goals)
            (r, g, b) = rgb_generator(1.0 * self.next_goal / len(goals))
            self.set_pen(r, g, b)
        
        x_dist = goals[self.next_goal][0] - pos.x
        y_dist = goals[self.next_goal][1] - pos.y
        
        desired_angle = math.atan2(y_dist, x_dist)
        err_angle = normalize_angle(desired_angle - pos.theta)
        vel.angular.z = 1 * err_angle
        vel.linear.x = 1.0
        
        self.velocity_publisher.publish(vel)
    
    def close_to_pos(self, cur_pos, next_pos):
        epsilon = 0.01
        return abs(next_pos[0] - cur_pos.x) < epsilon and (next_pos[1] - cur_pos.y) < epsilon
        
    
def main(args=None):
    print('Starting draw node...')
    rclpy.init(args=args)
    draw_node = DrawNode()
    rclpy.spin(draw_node)
    draw_node.destroy_node()

def normalize_vector(vec):
    magnitude = (vec[0]**2 + vec[1]**2)**0.5
    if magnitude == 0:
        return (0, 0)
    return (vec[0] / magnitude, vec[1] / magnitude)

def normalize_angle(angle):
    if (angle > math.pi):
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle

"""
rgb_generator takes a value between 0 and 1 (inclusive), and returns R, G, and B
values from 0 to 255. 0 and 1 will return (255, 0, 0) (red), and values in
between will return hues in between. See HSL color strips for more info.
"""
def rgb_generator(percent):
    (r, g, b) = colorsys.hsv_to_rgb(percent, 1, 1)
    r = math.floor(r * 255)
    g = math.floor(g * 255)
    b = math.floor(b * 255)
    return (r, g, b)

if __name__ == '__main__':
    main()
