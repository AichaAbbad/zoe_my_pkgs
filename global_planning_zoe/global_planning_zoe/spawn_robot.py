"""
Spawn robot from Rviz init pose !!!

"""

import sys
import time
from threading import Thread
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from map_simulator.srv import Spawn
from math import atan2

import rclpy
from rclpy.node import Node

class Spawner(Node):
    def __init__(self):
        super().__init__("spawn_robot")

        # connect to simulator spawn server
        self.srv = self.create_client(Spawn, '/simulator/spawn')
        while not self.srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')

        # prepare request
        self.request = Spawn.Request()
        self.request.robot_namespace = self.get_namespace()

        self.request.theta = 0.0
        self.request.radius = 0.4
        self.request.shape = self.request.SHAPE_SQUARE
        self.request.linear_noise = 0.0
        self.request.angular_noise = 0.0
        self.request.robot_color = [0,0,0]
        self.request.laser_color = [255, 0, 0]
        self.request.size = [1.6, 2.8, 0.1]
        self.request.force_scanner = False #self.declare_parameter('force_scanner', True)
        self.request.static_tf_odom = True  # self.declare_parameter('static_tf_odom', False)
        self.request.zero_joints = False #self.declare_parameter('zero_joints', False)


        # Subscribe to the initial position topic
        self.initial_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_position_callback, 10)

    def spawn_at(self, x, y, theta):

        self.request.x = x
        self.request.y = y
        self.request.theta = theta

        # send request
        self.future = self.srv.call_async(self.request)
        #rclpy.spin_until_future_complete(self, self.future)
        #response = self.future.result()
        #self.get_logger().info("successfully spawned robot in namespace %s" % self.request.robot_namespace)
        self.future.add_done_callback(self.res_callback)
            
    def res_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("successfully spawned robot in namespace %s" % self.request.robot_namespace)
        except Exception as e:
            self.get_logger().error('service call failed: %r' % (e,))
        
    def initial_position_callback(self,msg: PoseWithCovarianceStamped):
        # Callback function to handle initial position updates

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = 2*atan2(q.z, q.w)
        self.get_logger().info(f"Received initial position: ({x}, {y}, {theta})")
        self.spawn_at(x, y, theta)

# Main function
def main(args=None):
    rclpy.init(args=args)

    time.sleep(1)

    # Create node for simulation
    spawn = Spawner()

    # Spin indefinitely to allow respawn at any time
    rclpy.spin(spawn)

    # On shutdown...
    spawn.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()
