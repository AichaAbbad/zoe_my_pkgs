"""
This node: "DO PLANNING"

it gets initial and goal position from rviz and publish a path

publisher --> publish once (no freq included)

main_test_1 is an updated version :)

"""
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import osmnx  as ox   
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
#from global_planning_zoe_interface.srv import IntPose

class Globalplanning(Node):
    def __init__(self):
        super().__init__("main_test")

        self.map_pack = Openstreetmap()
        self.astar_pack = Astar()
        self.cnv = Conversions()
        self.graphpng = Graph()

        # Initialize initial and goal positions
        self.initial_positionx= 0.0
        self.initial_positiony= 0.0
        self.goal_positionx = 0.0
        self.goal_positiony = 0.0
        self.getin = False
        self.getgoal = False

        # Get the graph in local coordinates
        bbox = (47.25069, 47.24747, -1.54446, -1.55250)
        self.buildings, self.residential,self.service , self.ecn_graph_proj, self.ecn_graph = self.graphpng.get_graph_data(bbox)
        self.get_logger().info("graph {}".format(self.ecn_graph_proj))

        # Subscribe to the initial position topic
        self.initial_subscriber = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initial_position_callback, 10)

        # Subscribe to the goal position topic
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal_pose', self.goal_position_callback, 10)
        
        # publisher
        self.publisher = self.create_publisher(Path, 'path', 100)
        
        timer_period = 0.1  # 10hz
        self.timer = self.create_timer(timer_period, self.plan)

        self.broadcaster = TransformBroadcaster(self)

        
    def initial_position_callback(self,msg: PoseWithCovarianceStamped):
        # Callback function to handle initial position updates
        initial_position = msg.pose.pose.position
        self.initial_positionx = initial_position.x
        self.initial_positiony = initial_position.y
        self.getin = True
        self.get_logger().info("Received initial position: ({}, {})".format(self.initial_positionx, self.initial_positiony))


    def goal_position_callback(self,msg: PoseStamped):
        # Callback function to handle goal position updates
        goal_position = msg.pose.position
        self.goal_positionx = goal_position.x
        self.goal_positiony = goal_position.y
        self.getgoal = True
        self.get_logger().info("Received goal position: ({}, {})".format(self.goal_positionx, self.goal_positiony))
        
    
    def plan(self):
        if self.getgoal & self.getin :
            #########
            source_node = ox.nearest_nodes(self.ecn_graph_proj, self.initial_positionx,self.initial_positiony) # set initial node
            target_node = ox.nearest_nodes(self.ecn_graph_proj,self.goal_positionx,self.goal_positiony) # set target node
            self.get_logger().info(" source  & target nodes : ({}, {})".format(source_node, target_node))

            path = self.astar_pack.astar(self.ecn_graph_proj,source_node,target_node)
            #path = nx.shortest_path(self.ecn_graph_proj, source_node, target_node)
            self.get_logger().info("path {} {}".format(path, np.shape(path)))
                                   
            ## plotting the shortest path 
            my_dpi=96
            fig, ax = plt.subplots(1,1, figsize=(15,15),dpi = my_dpi)
            ax.set_facecolor('lightgrey')

            self.residential.plot(linewidth=10, ax=ax, color='w')  # residential streets are 3,5 m wide
            self.service.plot(linewidth=5, ax=ax, color='w') # service streets are 3 m wide
            self.buildings.plot(ax=ax, color = 'black', edgecolor = 'black', lw = 0.2)

            self.plot_path = [plt.plot(self.ecn_graph_proj.nodes[wpt]['x'], self.ecn_graph_proj.nodes[wpt]['y'], 'ro') for wpt in path]
            self.plot_nodes = plt.plot(self.ecn_graph_proj.nodes[source_node]['x'], self.ecn_graph_proj.nodes[source_node]['y'], 'bo',
            self.ecn_graph_proj.nodes[target_node]['x'], self.ecn_graph_proj.nodes[target_node]['y'], 'go')

            ax.set_xlim(0.0, 362.814)
            ax.set_ylim(0.0, 214.059)
            plt.show()
            
            mypath  = Path()

            for node_id in path:
                node = self.ecn_graph_proj.nodes[node_id]
                pose = PoseStamped()
                pose.pose.position.x = node['x']
                pose.pose.position.y = node['y']
                pose.pose.position.z = 0.0  # Assuming z-coordinate is 0
                pose.header.frame_id = 'map'
                mypath.poses.append(pose)
                mypath.header.frame_id = "map"

            self.publisher.publish(mypath)
            self.get_logger().info("path published")

            self.getgoal = False
            self.getin = False

        

# Main function
def main(args=None):
    rclpy.init(args=args)

    # Create node for simulation
    main_test = Globalplanning()
    
    # Spin indefinitely..
    rclpy.spin(main_test)

    # On shutdown...
    main_test.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()