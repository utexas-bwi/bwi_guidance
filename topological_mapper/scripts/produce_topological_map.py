#!/usr/bin/python

import roslib
roslib.load_manifest('topological_mapper')
from nav_msgs.msg import OccupancyGrid

from Tkinter import *

import sys
import os
import yaml
from PIL import Image
from tf.transformations import quaternion_from_euler

class BresenhamCanvas(Canvas):

    def setPixel(self, x, y, color):
        self.create_line(x, y, x+1, y+1, fill=color)

    def circle(self, cx, cy, radius, color):
        circle_provider = CircleProvider()
        points = circle_provider.circle(cx, cy, radius)
        for point in points:
            self.setPixel(point[0], point[1], color)

class CircleProvider:

    def circle(self, cx, cy, radius):
        self.points = []
        error = -radius
        x = radius
        y = 0
        while (x >= y):
            self.plot8points(cx, cy, x, y)
            error = error + y
            y = y + 1
            error = error + y
            if error >= 0:
                error = error - x
                self.plot8points(cx, cy, x, y) #4 connected
                x = x - 1
                error = error - x
        return self.points

    def plot8points(self, cx, cy, x, y):
        self.plot4points(cx, cy, x, y)
        if x != y:
            self.plot4points(cx, cy, y, x)
     
    def plot4points(self, cx, cy, x, y):
        self.setPixel(cx + x, cy + y);
        if x != 0:
            self.setPixel(cx - x, cy + y);
        if y != 0:
            self.setPixel(cx + x, cy - y);
        if x != 0 and y != 0:
            self.setPixel(cx - x, cy - y);
    
    def setPixel(self,x,y):
        self.points.append([x, y])

class MapLoader:

    def __init__(self, yaml_file):
        try:
            map_info = yaml.load(open(yaml_file, 'r'))
        except:
            sys.stderr.write("Unable to load yaml file for map: %s" %yaml_file)
            return

        resolution = map_info.get('resolution')
        origin = map_info.get('origin')
        negate = map_info.get('negate')
        occupied_thresh = map_info.get('occupied_thresh')
        free_thresh = map_info.get('free_thresh')

        image_file = map_info.get('image')
        if image_file[0] != '/': 
            yaml_file_dir = os.path.dirname(os.path.realpath(yaml_file))
            image_file = yaml_file_dir + '/' + image_file

        self.map = self.loadMapFromFile(image_file, resolution,
          negate, occupied_thresh, free_thresh, origin)

    def loadMapFromFile(self, image_file, resolution, negate, occupied_thresh, free_thresh, origin):
  
        map = OccupancyGrid()
    
        image = Image.open(image_file)
        pix = image.load()
    
        image_size = image.size
        map.info.width = image_size[0]
        map.info.height = image_size[1]
        map.info.resolution = resolution
    
        map.info.origin.position.x = origin[0]
        map.info.origin.position.y = origin[1]
        map.info.origin.position.z = 0
        q = quaternion_from_euler(0,0,origin[2])
        map.info.origin.orientation.x = q[0]
        map.info.origin.orientation.y = q[1]
        map.info.origin.orientation.z = q[2]
        map.info.origin.orientation.w = q[3]
    
        test_pxl = pix[0,0]
        if isinstance(test_pxl, (list, tuple)):
            is_multi_layer = True
            num_layers = len(test_pxl)
        else:
            is_multi_layer = False
            num_layers = 1
    
        map.data = [None] * image_size[0] * image_size[1]
        for j in range(image_size[1]):
            for i in range(image_size[0]):
                pxl = pix[i, j]
        
                if is_multi_layer:
                    color_average = sum(pxl) / num_layers
                else:
                    color_average = pxl

                if negate:
                    occ = color_average / 255.0;
                else:
                    occ = (255 - color_average) / 255.0;

                map_idx = map.info.width * (map.info.height - j - 1) + i
                if (occ > occupied_thresh):
                    map.data[map_idx] = 100
                elif (occ < free_thresh):
                    map.data[map_idx] = 0
                else:
                    map.data[map_idx] = -1
    
        return map
    
    def drawMap(self, canvas):
        for j in range(self.map.info.height):
            for i in range(self.map.info.width):
                map_idx = self.map.info.width * (self.map.info.height - j - 1) + i
                val = self.map.data[map_idx]
                if val == 100:
                    canvas.setPixel(i,j,"black")
                elif val == 0:
                    canvas.setPixel(i,j,"white")
                else:
                    canvas.setPixel(i,j,"gray")

class TopologicalMapper(MapLoader):

    def __init__(self, map_file):
        MapLoader.__init__(map_file)

    def drawVoronoiPoints(self, threshold):
        circle_provider = CircleProvider()

        farthest_j_vert = self.map.info.height - 1
        for j in range(self.map.info.height):
            if j > self.map.info.height/2:
                farthest_j_vert = 0
            farthest_i_vert = self.map.info.width - 1
            for i in range(self.map.info.width):
                if i > self.map.info.width/2:
                    farthest_i_vert = 0

                # check if this is free space
                map_idx = self.map.info.width * j + i
                if self.map.data[map_idx] != 0:
                    continue

                # otherwise compute the min and max radius to find an obstacle in
                min_radius = threshold
                max_radius = math.ceil(mat.sqrt(math.pow(i - farthest_i_vert, 2) + math.pow(j - farthest_j_vert, 2)))
                for radius in range(min_radius, max_radius):
                    points = circle_provider.circle(i, j, radius)
                    for points in point:
                        if point[0] >= 0 and point[0] < self.map.info.width and point[1] >= 0 and point[1] < self.map.info.height:
                            map_idx = self.map.info.width * j + i
                            if self.map.data[map_idx] != 100:



def run():

    root = Tk()
    canvas = BresenhamCanvas(root, width=600, height=1000)
    canvas.pack()
    
    # origin_x = 50
    # origin_y = 50
    # color = ["red", "blue", "green", "yellow", "purple"]
    # for radius in range(2,21):
    #     canvas.circle(origin_x, origin_y, radius, color[radius%5])

    if len(sys.argv) != 2:
        sys.stderr.write("USAGE: " + sys.argv[0] + " <yaml-map-file-location>")
        return

    map_loader = MapLoader(sys.argv[1])
    map_loader.drawMap(canvas)

    root.mainloop()
                
if __name__ == "__main__":
    run()
