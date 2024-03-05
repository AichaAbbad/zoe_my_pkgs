'''
this node is just a draft so far !

'''


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import osmnx as ox   
import networkx as nx
import numpy as np 
import logging
from .lib.openstreetmap import Openstreetmap
from .lib.astar import Astar
from .lib.graph import Graph, Conversions

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, TransformStamped
from map_simulator.srv import Spawn

from global_planning_zoe_interface.srv import IntPose 


class Spawn_robot(Node):
    def __init__(self):
        super().__init__("spawn_robot")

        self.initial_positionx= 0.0
        self.initial_positiony= 0.0

        # Subscribe to the initial position topic
        self.initial_subscriber = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initial_position_callback, 10)

        # Subscribe to the path
        self.path_subscriber = self.create_subscription(Path, 'path', self.path_callback, 10)

        # publisher
        self.odom_publisher = self.create_publisher(Odometry, 'r2d2/odom', 50)

        # service
        self.spawn(self.declare_parameter("static_tf", False).value)

        timer_period = 0.1  # 10hz
        self.timer = self.create_timer(timer_period, self.pub_callback)

    def initial_position_callback(self,msg: PoseWithCovarianceStamped):
        # Callback function to handle initial position updates
        initial_position = msg.pose.pose.position
        self.initial_positionx = initial_position.x
        self.initial_positiony = initial_position.y
        self.getin = True
        self.get_logger().info("Received initial position: ({}, {})".format(self.initial_positionx, self.initial_positiony))

    def spawn(self, static_tf):
        """
        robot = self.get_namespace().strip('/')

        spawner = self.create_client(Spawn, '/simulator/spawn')
        while not spawner.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')

        req = Spawn.Request()
        req.robot_namespace = self.get_namespace()
        
        req.radius = self.radius
        req.shape = req.SHAPE_CIRCLE
        req.static_tf_odom = static_tf

        self.future = spawner.call_async(req)
        self.get_logger().info(f'spawned robot {robot} in map_simulator')
         """
    
    def pub_callback(self):
        ## publish odometry data
            odom =  Odometry()
            odom.header.frame_id = "r2d2/odom"
            odom.child_frame_id = "r2d2/base_footprint"
            now = self.get_clock().now()
            odom.header.stamp = now.to_msg()
            odom.pose.pose.position.x = self.initial_positionx
            odom.pose.pose.position.y = self.initial_positiony
            odom.pose.pose.position.z = 0.0 # Assuming no rotation

            self.odom_publisher.publish(odom)
            self.get_logger().info("Published dato to r2d2: ({}, {})".format(odom.pose.pose.position.x, odom.pose.pose.position.y))





# Main function
def main(args=None):
    rclpy.init(args=args)

    # Create node for simulation
    spawn_robot = Spawn_robot()
    
    # Spin indefinitely..
    rclpy.spin(spawn_robot)

    # On shutdown...
    spawn_robot.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()