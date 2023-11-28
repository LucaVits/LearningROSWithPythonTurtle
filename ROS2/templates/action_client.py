import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import random

from geometry_msgs.msg import Twist, Pose

from turtle_interfaces.action import TurtleToGoals
from turtle_interfaces.msg import TurtleMsg

class TurtleClient(Node):
    def __init__(self):
        super().__init__('turtlebot_client')

        #### Action client ####
        self.action_cli = ActionClient(self, <action type>, <action name>)
        while not self.action_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TraveltoGoal action not available, waiting...')
        ####################################
        
    def goal_actioncall(self):
        
        goal_msg = TurtleToGoals.Goal()
        for i in range(5):
            goal = Pose()
            goal.position.x = float(random.randint(-1*self.screen.window_width()/2+20,self.screen.window_width()/2-20))
            goal.position.y = float(random.randint(-1*self.screen.window_height()/2+20,self.screen.window_height()/2-20))

            goal_msg.goal_poses.append(goal)
        
        self.action_response_future = self.action_cli.send_goal_async(goal_msg, feedback_callback=self.goal_feedback_cb)
        self.action_response_future.add_done_callback(self.goal_response_cb)
    
    def goal_response_cb(self, future):

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected')
            return
        
        self.get_logger().info('Goal accepted')

        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal Travel Done! Result:%d' % (result.ret))

    def goal_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        mid_goal = feedback.mid_goal_pose
        self.get_logger().info('Traveling to x:%d, y:%d' % (mid_goal.position.x, mid_goal.position.y))

def main(args=None):

    #initial ROS2
    rclpy.init(args=args)

    #initial turtle client
    cli_obj = TurtleClient()
    cli_obj.get_logger().info('Turtlebot Client Started!')

    # call the action
    cli_obj.goal_actioncall()
    
    # this is equal to rclpy.spin(cli_obj)
    while rclpy.ok():
        rclpy.spin_once(cli_obj)

    # Destory the node explicitly
    cli_obj.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()