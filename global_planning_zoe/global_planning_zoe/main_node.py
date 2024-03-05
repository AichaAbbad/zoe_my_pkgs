import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import osmnx  as ox   
import networkx as nx
import numpy as np 
import logging
from .lib.openstreetmap import Openstreetmap
from .lib.astar import Astar

logging.basicConfig(filename='main_code.log', level=logging.INFO)

class Globalplanning(Node):
    def __init__(self):
        super().__init__("main_node")

        map_pack = Openstreetmap()
        astar_pack = Astar()

        location = (47.2489965, -1.5473425) # add your location
        graph = map_pack.get_map(location) # get your graph 
        source_node = ox.nearest_nodes(graph, -1.5480, 47.2489) # set initial node
        target_node = ox.nearest_nodes(graph, -1.5644, 47.2062) # set target node
        print(source_node)
        #print(graph)
        #ox.save_graphml(graph)

        map_pack.get_shortest_path(graph,source_node,target_node)
        logging.info("Plot the shortest path !")
        astar_pack.astar(graph,source_node,target_node)
        logging.info("Plot the shortest path with A* !")

# Main function
def main(args=None):
    rclpy.init(args=args)

    # Create node for simulation
    main_node = Globalplanning()
    
    # Spin indefinitely..
    #rclpy.spin(main_node)

    # On shutdown...
    main_node.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()