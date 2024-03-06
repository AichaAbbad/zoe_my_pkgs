"""
Spawn robot from Rviz init pose !!!

"""

import sys
import time
from threading import Thread
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from map_simulator.srv import Spawn

import rclpy
from rclpy.node import Node

class Spawner(Node):
    def __init__(self):
        super().__init__("spawn_robot")

        # Initialize initial and goal positions
        self.initial_positionx= 0.0
        self.initial_positiony= 0.0
        self.getin = False

        # spawn the robot in Rviz
        self.srv = self.create_client(Spawn, '/simulator/spawn')
        while not self.srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...') 

        # Subscribe to the initial position topic
        self.initial_subscriber = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initial_position_callback, 10)

    def send_request(self):
        self.request = Spawn.Request()
        self.request.robot_namespace = self.get_namespace()
        self.request.x = self.x
        self.request.y = self.y
        self.request.theta =  0.0
        self.request.radius = 0.4

        shape = self.declare_parameter('shape', 'rectangle')
        if shape == 'square':
            self.request.shape = self.request.SHAPE_SQUARE
        elif shape == 'rectangle':
            self.request.shape = self.request.SHAPE_RECTANGLE
        else:
            self.request.shape = self.request.SHAPE_CIRCLE

        self.request.linear_noise = 0.0
        self.request.angular_noise = 0.0

        robot_color_param = self.declare_parameter('robot_color', [0, 0, 0])
        robot_color = robot_color_param.value
        if len(robot_color) != 3:
            robot_color = [0, 0, 0]
        self.request.robot_color = robot_color

        laser_color_param = self.declare_parameter('laser_color', [255, 0, 0])
        laser_color = laser_color_param.value
        if len(laser_color) != 3:
            laser_color = [255, 0, 0]
        self.request.laser_color = laser_color

        size_param= self.declare_parameter('size', [0, 0, 0])
        size = size_param.value
        if len(size) != 3 or not all(isinstance(s, float) for s in size):
            size = [0.0, 0.0, 0.0]
        self.request.size = size

        self.request.force_scanner = False #self.declare_parameter('force_scanner', True)
        self.request.static_tf_odom = True  # self.declare_parameter('static_tf_odom', False)
        self.request.zero_joints = False #self.declare_parameter('zero_joints', False)

        # send request
        self.future = self.srv.call_async(self.request)
        #rclpy.spin_until_future_complete(self, self.future)
        #response = self.future.result()
        #self.get_logger().info("successfully spawned robot in namespace %s" % self.request.robot_namespace)
        self.future.add_done_callback(self.callback)
            
    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("successfully spawned robot in namespace %s" % self.request.robot_namespace)
        except Exception as e:
            self.get_logger().error('service call failed: %r' % (e,))

        
    def initial_position_callback(self,msg: PoseWithCovarianceStamped):
        # Callback function to handle initial position updates
        initial_position = msg.pose.pose.position
        self.initial_positionx = initial_position.x
        self.initial_positiony = initial_position.y
        self.getin = True
        self.get_logger().info("Received initial position: ({}, {})".format(self.initial_positionx, self.initial_positiony))

    def set_init(self):
        #if self.getin:
        self.x = self.initial_positionx
        self.y = self.initial_positiony
        self.get_logger().info("Received position: ({}, {})".format(self.x, self.y))
        self.send_request()
        #else:
            #self.get_logger().warn("Initial position not received yet. Unable to set_init.")



# Main function
def main(args=None):
    rclpy.init(args=args)

    time.sleep(1)

    # Create node for simulation
    spawn = Spawner()

    spin_thread = Thread(target=rclpy.spin, args=(spawn,))
    spin_thread.start()

    spawn.set_init()

    # Spin indefinitely..
    #rclpy.spin(spawn)

    # On shutdown...
    spawn.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()