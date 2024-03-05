'''

The following code preforms the following taskts:
    - get_map(): Get the OpenStreetMap of the input location
    - get_shortest_path(): Compute the shortest path to the goal

INPUT: * Location 
       * source_node
       * target_node

OUUTPUT: * graph
         * shortest_path

'''

## Imports
import matplotlib.pyplot as plt
import osmnx  as ox   
import networkx as nx
import numpy as np 
import logging
from pyrosm import OSM, get_data

# log file 
logging.basicConfig(filename='openstreetmap.log', level=logging.INFO)

## ---------- Loading and visualition of OMS map ---------- ##


class Openstreetmap():

    def get_map(self, location):
        # We load the map of ECN
        mode = "bike" # to get the private roads of ECN
        ox.settings.log_console=False
        ox.settings.use_cache=True
        graph = ox.graph_from_point(location, dist=250, simplify=False, network_type=mode )
        logging.info('Get the graph !')
        # Plot the graph
        fig, ax = ox.plot_graph(graph,node_color='g', edge_linewidth=1, edge_alpha=1.0)
        logging.info('Plot the map !')
        return graph
    
    ## ---------- Compute the shortest path ---------- ##
    def get_shortest_path(self, graph,source_node,target_node):
        # Shortest path (by distance)
        route = nx.shortest_path(graph, source_node, target_node, weight="length")
        logging.info('Calculate the shortest path !')
        # Plot
        fig1, ax1 = ox.plot_graph_route(graph, route, route_linewidth=6, node_size=0, bgcolor='k')
        logging.info("Plot the shortest path !")
        return route