import rclpy
from rclpy.node import Node

from turtle_interfaces.srv import SetColor

class TurtleClient(Node):
    def __init__(self):
        super().__init__('service_client')

        #### Color service client ####
        self.color_cli = self.create_client(<service type>, <service name>)
        while not self.color_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Color service not available, waiting...')
        self.color_req = SetColor.Request()

        self.server_call = False
        #################################
    
    def color_srvcall(self):

        col = 'cyan'
        
        self.color_req.color = col
        self.server_call = True
        self.service_future = self.color_cli.call_async(self.color_req)

def main(args=None):

    #initial ROS2
    rclpy.init(args=args)

    #initial turtle client
    cli_obj = TurtleClient()
    cli_obj.get_logger().info('Turtlebot Client Started!')
    
    # call the service
    cli_obj.color_srvcall()

    while rclpy.ok():
        rclpy.spin_once(cli_obj)

        if cli_obj.service_future.done() and cli_obj.server_call:
            cli_obj.server_call = False
            try:
                response = cli_obj.service_future.result()
            except Exception as e:
                cli_obj.get_logger().info('Server call failed')
            else:
                cli_obj.get_logger().info('Server call success: %d' % (response.ret))
                break

    # Destory the node explicitly
    cli_obj.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()