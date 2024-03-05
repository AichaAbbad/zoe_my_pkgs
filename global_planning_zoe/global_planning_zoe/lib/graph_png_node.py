'''
This node aims to generate the graph and a .png image
of a given location from OSM.

INPUTS: * Location ( bounding box)

OUTPUTS: * Graph
         * ".png" image

'''

#%matplotlib inline
import matplotlib.pyplot as plt
import contextily as cx
import geopandas
import pandas as pd
import rasterio
from rasterio.plot import show as rioshow
import matplotlib.pyplot as plt
import xyzservices.providers as xyz
import osmnx  as ox   
import networkx as nx
import numpy as np
from shapely.geometry import Point
import cv2 as cv


class Getgraphpng():
    # Location of ECN
    ''' Later this should be as input from main_test.py'''
    #bbox = (47.25069, 47.24747, -1.54446, -1.55250)

    def get_graph_data(self, bbox):
        north, south, east, west = bbox
        # extract the from
        ecn_graph = ox.graph_from_bbox(north, south, east, west, custom_filter='["highway"~"residential|service"]')
        ecn_graph = ox.utils_graph.remove_isolated_nodes(ecn_graph)
        ecn_nodes, ecn_streets  = ox.graph_to_gdfs(ecn_graph)

        # get buildings, service, and residential
        residential = ecn_streets[ecn_streets['highway'] == 'residential']
        service = ecn_streets[ecn_streets['highway'] == 'service']

        # get the geodata features of the map
        tags = {"building": True,"barrier":True, "leisure":True, "landuse":"grass", "natural":"wood" , "highway": "footpath"}
        buildings = ox.features_from_bbox(north, south, east, west, tags=tags)
        buildings = buildings.reset_index()
        # sometimes building footprints are represented by Points, let's disregard them
        buildings = buildings[(buildings.geometry.geom_type == 'Polygon') | (buildings.geometry.geom_type == 'MultiPolygon')]
        buildings.crs

        #print("get_graph_data")
        return buildings,residential,service, ecn_graph, ecn_nodes, ecn_streets
    
    def get_web_mercator_coord(self,bbox):
        north, south, east, west = bbox
        s = geopandas.GeoSeries([Point(west,south), Point(east,north)])
        latitude = 47.24737 * np.pi / 180
        zoomlevel = 17
        C = 40075016.686
        Spixel = C * np.cos(latitude) / 2**(zoomlevel + 8)
        s.crs = "epsg:4326"

        #print("get_web_mercator_coord")
        return Spixel,s
    
    def lonlat_to_web_mercator(self,bbox):
        buildings,residential,service, ecn_graph,ecn_nodes, ecn_streets = self.get_graph_data(bbox)
        Spixel,s = self.get_web_mercator_coord(bbox)

        # convert lanes and building to web mercator
        residential = residential.to_crs(epsg=3857) # convert to web mercator
        service = service.to_crs(epsg=3857) # convert to web mercator
        buildings = buildings.to_crs(epsg=3857) # convert to web mercator
        s = s.to_crs(epsg=3857) # convert to web mercator

        #print("lonlat_to_web_mercator")
        return buildings,residential,service,ecn_graph,s,Spixel

    def get_graph(self,bbox): # 
        # the graph is in web mercator coordinates
        buildings,residential,service,ecn_graph,s,Spixel = self.lonlat_to_web_mercator(bbox)
        ecn_graph_proj = ox.project_graph(ecn_graph, to_crs='EPSG:3857')
        self.plot_map(residential,service,buildings)
        #ox.plot_graph(ecn_graph_proj, ax=ax, node_size=50, edge_linewidth=5, edge_color='white', node_color='white')
        ecn_graph_proj.nodes().data()

        #print("get_graph")
        return ecn_graph_proj
    
    def translate(self,graph : nx.MultiDiGraph, xoff, yoff):
        for node, data in graph.nodes(data=True):
            data['x'] += xoff
            data['y'] += yoff
        #print("translate")
        return graph
    
    def plot_map(self,residential,service,buildings):
        plt.rcParams["axes.facecolor"] = "lightgrey"
        my_dpi=96
        fig, ax = plt.subplots(1,1, figsize=(15,15), dpi=my_dpi)
        ax.set_facecolor('lightgrey')
        residential.plot(linewidth=10, ax=ax, color='w')  # residential streets are 3,5 m wide
        service.plot(linewidth=5, ax=ax, color='w') # service streets are 3 m wide
        buildings.plot(ax=ax, color = 'black', edgecolor = 'black', lw = 0.2)

        #print("plot_map")
        return plt

    def graph_in_local(self,bbox):
        
        buildings,residential,service,ecn_graph,s,Spixel= self.lonlat_to_web_mercator(bbox)

        ecn_graph_proj = self.get_graph(bbox)

        ecn_graph_proj = self.translate(ecn_graph_proj, 172823.509, -5982561.437)
        # transform to have Point(0,0) at the bottom left corner -172823.509 5982561.43)
        buildings = buildings.translate(xoff=172823.509, yoff=-5982561.437)
        residential = residential.translate(xoff=172823.509, yoff=-5982561.437)
        # residential = residential.buffer(Spixel)
        service = service.translate(xoff=172823.509, yoff=-5982561.437)
        # service = service.buffer(Spixel)
        s = s.translate(xoff=172823.509, yoff=-5982561.437)
        plt = self.plot_map(residential,service,buildings)
        plt.savefig('map.png')
        plt.show()
        self.save_yaml(Spixel)

        #print("graph_in_local")
        return ecn_graph_proj,buildings,residential,service
    
    def save_yaml(self,Spixel):
        map = {
            "image": "map.png",
            "resolution": str(Spixel),
            "origin": [0.0, 0.0, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196
        }
        return None

class Conversions():
    
    def to_local(self,lon, lat , lon_origin , lat_origin):
        # convert to web mercator
        origin = geopandas.GeoSeries(Point(lon_origin , lat_origin))
        origin.crs = "epsg:4326"    
        point = geopandas.GeoSeries(Point(lon , lat))
        point.crs = "epsg:4326"
        origin = origin.to_crs(epsg=3857)
        point = point.to_crs(epsg=3857)

        # translate to origin
        point = point.translate(xoff=-origin[0].x, yoff=-origin[0].y)
        return point[0].x, point[0].y
        #return point
    
    def to_lon_lat(self,x : float, y : float, lon_origin, lat_origin):
        # convert to web mercator
        origin = geopandas.GeoSeries(Point(lon_origin , lat_origin))
        origin.crs = "epsg:4326" 
        origin = origin.to_crs(epsg=3857)
        
        # translate to origin
        point = geopandas.GeoSeries(Point(x + origin[0].x, y + origin[0].y))
        point.crs = "epsg:3857"
        point = point.to_crs(epsg=4326)

        return point[0].x, point[0].y


'''
bbox = (47.25069, 47.24747, -1.54446, -1.55250)
func = Getgraphpng()
func.graph_in_local(bbox)

x, y = Conversions.to_local(-1.55250,47.24747, -1.55250,47.24747)
print(x, y)

lon, lat= Conversions.to_lon_lat(x, y, -1.55250,47.24747)
print(lon, lat)
'''