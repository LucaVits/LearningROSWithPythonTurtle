import colorsys
import math

import rclpy
import rclpy.action
from geometry_msgs.msg import Twist
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.action.client import GoalStatus
from rclpy.node import Node
from turtlesim.action import RotateAbsolute
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute

star = [(5.0, 1.0), (7.0, 8.0), (1.0, 7.0), (9.0, 7.0), (3.0, 8.0)]

zigzag = [(1.0, 1.0), (2.0, 2.0), (3.0, 1.0), (4.0, 2.0), (5.0, 1.0)]

goals = star

class DrawNode(Node):
    def __init__(self):
        super().__init__('draw_node_actions')
        self.next_goal = 0
        self.rotate_goal_future: Future = Future()
        self.rotate_remaining = 0

        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.rotate_client = ActionClient(self, RotateAbsolute, '/turtle1/rotate_absolute')

        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.teleport(goals[0])  # TP to starting point
    
    def rotate_to_goal(self, angle):
        goal = RotateAbsolute.Goal()
        goal.theta = angle
        return self.rotate_client.send_goal_async(goal)
        # We DO care what the result is: return it
        
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
            print('Switching to next goal.')
            self.next_goal = (self.next_goal + 1) % len(goals)
            (r, g, b) = rgb_generator(1.0 * self.next_goal / len(goals))
            self.set_pen(r, g, b)
            x_dist = goals[self.next_goal][0] - pos.x
            y_dist = goals[self.next_goal][1] - pos.y
            desired_angle = math.atan2(y_dist, x_dist)
            self.rotate_goal_future = self.rotate_to_goal(desired_angle)

        goal_handle: ClientGoalHandle = self.rotate_goal_future.result()
        if goal_handle is None or goal_handle.status != GoalStatus.STATUS_SUCCEEDED:
            print('Wating for turn to finish...')
        else:
            print('Moving!')
            vel.linear.x = 1.0
            self.velocity_publisher.publish(vel)

    def close_to_pos(self, cur_pos, next_pos):
        epsilon = 1
        return abs(next_pos[0] - cur_pos.x) < epsilon and (next_pos[1] - cur_pos.y) < epsilon


def main(args=None):
    print('Starting draw node with actions...')
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
rgb_generator takes a value between 0 and 1 (inclusive), and returns R, G, and
B values from 0 to 255. 0 and 1 will return (255, 0, 0) (red), and values in
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
