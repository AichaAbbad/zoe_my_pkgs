'''

The following code preforms the following taskts:
    - heuristic(): Heuristic function
    - astar(): Compute the shortest path to the goal using A* algorithm

INPUT: * Location 
       * source_node
       * target_node
       * heuristic

OUUTPUT: * shortest_path

'''

#### --------- Global Planning with A* --------- ####

## Imports
import matplotlib.pyplot as plt
import osmnx  as ox   
import networkx as nx
import numpy as np 
import logging
from .openstreetmap import Openstreetmap
from pyrosm import OSM, get_data

# log file 
logging.basicConfig(filename='astar.log', level=logging.INFO)

# ---------  A* implementation --------- #

class Astar():

    def heuristic(self, node1, node2):
        logging.info('Heuristic function: Compute distance')
        return ((node2 - node1)**2)**0.5

    def astar(self, graph,start_pos,goal_pos):
        logging.info('Implement A* algorithm !')

        open_set = {start_pos}
        came_from = {}
        g_score = {node: float('inf') for node in graph.nodes}
        f_score = {node: float('inf') for node in graph.nodes}
        g_score[start_pos] = 0
        f_score[start_pos] = self.heuristic(start_pos, goal_pos)

        while open_set:
            current_node = min(open_set, key=lambda node: f_score[node])
            if current_node == goal_pos:
                logging.info('Current node = goal node')
                path = []
                while current_node in came_from:
                    path.append(current_node)
                    current_node = came_from[current_node]
                path.append(start_pos)
                path.reverse()
                logging.info('Goal reached : Return the path !')
                if path:
                    logging.info('Path found !')
                    print(path)
                    #fig, ax = ox.plot_graph_route(graph, path, route_color='b', node_color='g', edge_linewidth=1, edge_alpha=1.0)
                    #logging.info('Plot the path !')
                    #plt.show()
                else:
                    logging.info('Path not found !')
                return path
            
            open_set.remove(current_node)

            logging.info('Current node != goal node')
            for neighbor in graph.neighbors(current_node):
                temp_g_score = g_score[current_node] + graph[current_node][neighbor].get('length',1)
                if temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current_node
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] =  g_score[neighbor] + self.heuristic(neighbor, goal_pos)
                    if neighbor not in open_set:
                        open_set.add(neighbor)    
        return None
