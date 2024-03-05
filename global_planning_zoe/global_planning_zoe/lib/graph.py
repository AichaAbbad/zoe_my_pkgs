'''

This code aims to extract OSM map data starting from a given bounding box through
the "Graph" class.

Graph():
    INPUT:
        * bbox = (north, south, east, west) with (lon, lat) coordinates

    OUTPUT:
        * graph in local coordinates (ecn_graph_proj)
        * graph in (lon, lat) coordinates (ecn_graph)
        * buildings in local coordinates
        * residential in local coordinates
        * service in local coordinates
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
import yaml

class Graph():
    # Location of ECN

    def get_graph_data(self, bbox):

        self.north, self.south, self.east, self.west = bbox
        # extract the from
        self.ecn_graph = ox.graph.graph_from_bbox(self.north, self.south, self.east, self.west, custom_filter='["highway"~"residential|service"]')
        self.ecn_graph = ox.utils_graph.remove_isolated_nodes(self.ecn_graph)
        self.ecn_nodes, self.ecn_streets  = ox.graph_to_gdfs(self.ecn_graph)

        # get buildings, service, and residential
        self.residential = self.ecn_streets[self.ecn_streets['highway'] == 'residential']
        self.service = self.ecn_streets[self.ecn_streets['highway'] == 'service']

        # get the geodata features of the map
        tags = {"building": True,"barrier":True, "leisure":True, "landuse":"grass", "natural":"wood" , "highway": "footpath"}
        self.buildings = ox.features_from_bbox(self.north, self.south, self.east, self.west, tags=tags)
        self.buildings = self.buildings.reset_index()
        # sometimes building footprints are represented by Points, let's disregard them
        self.buildings = self.buildings[(self.buildings.geometry.geom_type == 'Polygon') | (self.buildings.geometry.geom_type == 'MultiPolygon')]
        self.buildings.crs

        ###### Get web mercator coordinates conversion
        self.s = geopandas.GeoSeries([Point(self.west,self.south), Point(self.east,self.north)])
        latitude = 47.24747 * np.pi / 180
        zoomlevel = 18
        C = 40075016.686
        self.Spixel = C * np.cos(latitude) / 2**(zoomlevel + 8)
        self.s.crs = "epsg:4326"
        
        ##### convert lanes and building to web mercator (graph in web merc)
        self.residential = self.residential.to_crs(epsg=3857) # convert to web mercator
        self.service = self.service.to_crs(epsg=3857) # convert to web mercator
        self.buildings = self.buildings.to_crs(epsg=3857) # convert to web mercator
        self.s = self.s.to_crs(epsg=3857) # convert to web mercator

        self.ecn_graph_proj = ox.project_graph(self.ecn_graph, to_crs='EPSG:3857')
        self.ecn_graph_proj.nodes().data()

        ####
        self.ecn_graph_proj = self.translate(self.ecn_graph_proj, xoff=172823.509, yoff=-5982561.437)
        self.ecn_graph_proj = self.scale_t(self.ecn_graph_proj)

        # transform to have Point(0,0) at the bottom left corner -172823.509 5982561.43)
        self.buildings = self.buildings.translate(xoff=172823.509, yoff=-5982561.437)
        self.buildings = self.buildings.scale(xfact=self.Spixel, yfact=self.Spixel, zfact=1.0, origin = (0,0))
 
        self.residential = self.residential.translate(xoff=172823.509, yoff=-5982561.437)
        self.residential = self.residential.scale(xfact=self.Spixel, yfact=self.Spixel, zfact=1.0, origin = (0,0))

        self.service = self.service.translate(xoff=172823.509, yoff=-5982561.437)
        self.service = self.service.scale(xfact=self.Spixel, yfact=self.Spixel, zfact=1.0, origin = (0,0))

        self.s = self.s.translate(xoff=172823.509, yoff=-5982561.437)
        self.s  = self.s .scale(xfact=self.Spixel, yfact=self.Spixel, zfact=1.0, origin = (0,0))
        print(self.s.head())

        #### plot
        plt.rcParams["axes.facecolor"] = "lightgrey"
        my_dpi=96
        fig, ax = plt.subplots(1,1, figsize=(15,15), dpi= my_dpi)
        ax.set_facecolor('lightgrey')
        
        self.residential.plot(linewidth=10, ax=ax, color='w')  # residential streets are 3,5 m wide
        self.service.plot(linewidth=5, ax=ax, color='w') # service streets are 3 m wide
        self.buildings.plot(ax=ax, color = 'black', edgecolor = 'black', lw = 0.2)

        ax.set_xlim(0.0, 362.814)
        ax.set_ylim(0.0, 214.059)
        plt.show()

        ## yaml
        map = {
            "image": "map.png",
            "resolution": str(self.Spixel),
            "origin": [0.0, 0.0, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196
        }

        print(self.Spixel)
        return self.buildings, self.residential,self.service , self.ecn_graph_proj, self.ecn_graph

    def translate(self,graph : nx.MultiDiGraph, xoff, yoff):
        for node, data in graph.nodes(data=True):
            data['x'] += xoff
            data['y'] += yoff
        return graph
    
    def scale_t(self,graph: nx.MultiDiGraph):
        for node, data in graph.nodes(data=True):
            # Convert pixel coordinates to meters
            data['x'] *= self.Spixel
            data['y'] *= self.Spixel
        return graph


### This part is still under development 

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
