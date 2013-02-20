#!/usr/bin/python

import roslib
roslib.load_manifest('topological_mapper')
from nav_msgs.msg import OccupancyGrid

from Tkinter import *

import sys
import os
import yaml
import math

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
    
    def drawMap(self, canvas, map=None):
        if not map:
            map = self.map
        for j in range(map.info.height):
            for i in range(map.info.width):
                map_idx = map.info.width * (map.info.height - j - 1) + i
                val = map.data[map_idx]
                if val == 100:
                    canvas.setPixel(i,j,"black")
                elif val == 0:
                    canvas.setPixel(i,j,"white")
                else:
                    canvas.setPixel(i,j,"gray")

class TopologicalMapper(MapLoader):

    def __init__(self, map_file):
        MapLoader.__init__(self, map_file)

    def findVoronoiPoints(self, threshold):
        threshold = int(math.ceil(threshold / self.map.info.resolution))

        #precompute inflated map
        self.inflated_map = OccupancyGrid()
        self.inflated_map.info = self.map.info
        self.inflated_map.data = [0] * len(self.map.data)
        for j in range(self.map.info.height):
            low_j = max(0, j - (threshold - 1))
            high_j = min(self.map.info.height - 1, j + (threshold - 1))
            for i in range(self.map.info.width):
                obstacle_found = False
                if low_j != j - (threshold - 1) or high_j != j + (threshold - 1) or i < threshold - 1 or i >= self.map.info.width - (threshold - 1):
                    obstacle_found = True
                else:
                    for j_kernel in range(low_j, high_j + 1):
                        j_diff = j - j_kernel
                        i_diff = math.sqrt((threshold - 1)*(threshold - 1) - j_diff * j_diff)
                        low_i = max(0, int(math.floor(i - i_diff)))
                        high_i = min(self.map.info.width - 1, int(math.ceil(i + i_diff)))
                        for i_kernel in range(low_i, high_i + 1):
                            kernel_idx = self.map.info.width * j_kernel + i_kernel
                            if self.map.data[kernel_idx] != 0:
                                #print "in"
                                obstacle_found = True
                                break
                        if obstacle_found:
                            break
                if obstacle_found:
                    inflated_map_idx = self.map.info.width * j + i
                    self.inflated_map.data[inflated_map_idx] = 100

        # for each point not close enough to an obstacle, compute voronoiness
        self.voronoi_points = {}
        for j in range(self.map.info.height):
            for i in range(self.map.info.width):
                basis_points = []
                basis_distance = 0
                inflated_map_idx = self.inflated_map.info.width * j + i
                if self.inflated_map.data[inflated_map_idx] != 0: #this point within threshold distance of obstacle, cannot be a voronoi point
                    continue
                for box in range (threshold,max(self.map.info.height,self.map.info.width)):

                    # we have come too far since finding the first point, that is it for this one
                    if len(basis_points) != 0 and box > basis_distance + 1:
                        break

                    #find new points we are comparing against
                    low_j = max(0, j - box)
                    high_j = min(self.map.info.height - 1, j + box)
                    low_i = max(0, i - box)
                    high_i = min(self.map.info.width - 1, i + box)
                    box_points = [(i_box, j_box) for i_box in (low_i, high_i) for j_box in range(low_j, high_j + 1)] #left and right + corners
                    box_points.extend([(i_box, j_box) for i_box in range(low_i + 1, high_i) for j_box in (low_j, high_j)]) # top and bottom
                    obstacle_points = [(box[0], box[1]) for box in box_points if self.map.data[self.map.info.width * box[1] + box[0]] != 0]

                    #if no obstacles are found, proceed to next box
                    if len(obstacle_points) == 0: 
                        continue

                    # obstacles have been found, sort!
                    obstacle_distances = [(pt[0], pt[1], math.sqrt((i-pt[0])*(i-pt[0]) + (j-pt[1])*(j-pt[1]))) for pt in obstacle_points]
                    sorted_distances = sorted(obstacle_distances, key=lambda point: point[2])
                    #print "------------------"
                    #print i,j,sorted_distances

                    #if no basis points exist, the closest obstacle becomes the first basis point
                    if len(basis_points) == 0:
                        obstacle = sorted_distances[0]
                        basis_points.append((obstacle[0], obstacle[1], obstacle[2]))
                        basis_distance = obstacle[2]
                        if len(sorted_distances) == 1: # no more points, no need to continue
                            continue
                        else: #more points, remove the basis point and continue
                            sorted_distances.pop(0)

                    #obstacles exist that need to be compared to existing basis points
                    for obstacle in sorted_distances:
                        if obstacle[2] > basis_distance + 1: #no more useful obstacles
                            break
                        #check if this basis point is not too close to existing basis points
                        should_add = True
                        for existing_point in basis_points:
                            if math.sqrt(math.pow(existing_point[0] - obstacle[0], 2) + math.pow(existing_point[1] - obstacle[1], 2)) < 2.0 * threshold:
                                should_add = False
                                break
                        if should_add:
                            basis_points.append((obstacle[0], obstacle[1], obstacle[2]))

                    #closer basis points may be available. recompute the basis distance and filter
                    basis_points = sorted(basis_points, key=lambda point: point[2])
                    basis_distance = basis_points[0][2]
                    basis_points = [point for point in basis_points if point[2] <= basis_distance + 1]

                if len(basis_points) >= 2:
                    self.voronoi_points[(i,j)] = basis_points, basis_distance

    # def findVoronoiPoints(self, threshold):
    #     circle_provider = CircleProvider()

    #     # precompute circle offsets
    #     # first get the check if a point is too close too an obstacle
    #     circle_points = circle_provider.circle(0, 0, threshold - 1)
    #     too_close = [() for i in range(2 * threshold - 1)]
    #     for y in range(len(too_close)):
    #         y_loc = y - (threshold - 1)
    #         x_locs = [point[0] for point in circle_points if point[1] == y_loc]
    #         too_close[y] = (min(x_locs), max(x_locs) + 1)
    #     # then precompute circle offsets for correct thresholds
    #     max_radius = 200#int(math.ceil(math.sqrt(math.pow(self.map.info.width, 2) + math.pow(self.map.info.height, 2))))
    #     circle_range = {}
    #     for radius in range(threshold, max_radius+1):
    #         circle_points = circle_provider.circle(0, 0, radius)
    #         circle_range[radius] = [() for i in range(2 * radius + 1)]
    #         for y in range(len(circle_range[radius])):
    #             y_loc = y - radius
    #             x_left = [point[0] for point in circle_points if point[1] == y_loc and point[0] < 0]
    #             x_right = [point[0] for point in circle_points if point[1] == y_loc and point[0] >= 0]
    #             #print circle_points, x_left, x_right, y_loc, radius
    #             circle_range[radius][y] = (min(x_left), max(x_left) + 1, min(x_right), max(x_right) + 1)

    #     self.voronoi_points = {}
    #     farthest_j_vert = self.map.info.height - 1
    #     for j in range(self.map.info.height):
    #         print j
    #         if j > self.map.info.height/2:
    #             farthest_j_vert = 0
    #         farthest_i_vert = self.map.info.width - 1
    #         for i in range(self.map.info.width):
    #             if i > self.map.info.width/2:
    #                 farthest_i_vert = 0

    #             # check if this is free space
    #             map_idx = self.map.info.width * j + i
    #             if self.map.data[map_idx] != 0:
    #                 continue

    #             # check if this point is really close to an obstacle (can still be improved by using an inflated map)
    #             allow_point = True
    #             for y in range(len(too_close)):
    #                 y_loc = y - (threshold - 1) + j
    #                 x_min, x_max = too_close[y]
    #                 for x in range(x_min, x_max):
    #                     x_loc = x + i
    #                     if x_loc >= 0 and x_loc < self.map.info.width and y_loc >= 0 and y_loc < self.map.info.height: # inside map boundaries
    #                         map_idx = self.map.info.width * y_loc + x_loc
    #                         if self.map.data[map_idx] != 0: # not free space
    #                             allow_point = False
    #                             break
    #                     else:
    #                         allow_point = False
    #                         break

    #                 if not allow_point:
    #                     break
    #             if not allow_point:
    #                 continue

    #             # otherwise compute the min and max radius to find an obstacle in
    #             min_radius = threshold
    #             max_radius = int(math.ceil(math.sqrt(math.pow(i - farthest_i_vert, 2) + math.pow(j - farthest_j_vert, 2))))

    #             # reset basis points and hunt for them
    #             basis_points = []
    #             last_round = False
    #             for radius in range(min_radius, max_radius):
    #                 
    #                 # some basis points may have already been found, but need to consider one more loop for discretization errors
    #                 last_round = len(basis_points) != 0
    #                  
    #                 # see if an obstacle exists at this distance
    #                 
    #                 for y in range(len(circle_range[radius])):
    #                     y_loc = y - radius + j
    #                     x_left_min, x_left_max, x_right_min, x_right_max = circle_range[radius][y]
    #                     x_range = range(x_left_min, x_left_max)
    #                     x_range.extend(range(x_right_min, x_right_max))
    #                     for x in x_range:
    #                         x_loc = x + i
    #                         # if j == 71 and i == 62 and radius > 15:
    #                         #     print radius,[x_loc, y_loc], 
    #                         if x_loc >= 0 and x_loc < self.map.info.width and y_loc >= 0 and y_loc < self.map.info.height:
    #                             map_idx = self.map.info.width * y_loc + x_loc
    #                             if self.map.data[map_idx] != 0:
    #                                 # if j == 71 and i == 62 and radius > 15:
    #                                 #     print "in", 
    #                                 # should we add a new basis point? should be atleast 2.0 * threshold away
    #                                 should_add = True
    #                                 for existing_point in basis_points:
    #                                     if math.sqrt(math.pow(existing_point[0] - x_loc, 2) + math.pow(existing_point[1] - y_loc, 2)) <= 1.8 * threshold:
    #                                         should_add = False
    #                                         break
    #                                 if should_add:
    #                                     # if j == 71 and i == 62 and radius > 15:
    #                                     #     print "appended",x 
    #                                     basis_points.append((x_loc, y_loc))
    #                         else:
    #                             last_round = True

    #                 if last_round:
    #                     break

    #             # if more than 2 points, then this is a voronoi point
    #             if len(basis_points) >= 2:
    #                 self.voronoi_points[(i, j)] = radius, basis_points


    def drawInflatedMap(self, canvas):
        self.drawMap(canvas, self.inflated_map)

    def drawVoronoiPoints(self, canvas):
        #print self.voronoi_points
        for point in self.voronoi_points.keys():
            canvas.setPixel(point[0], self.map.info.height - 1 - point[1], "red")

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

    mapper = TopologicalMapper(sys.argv[1])
    print "Drawing map... ",
    mapper.drawMap(canvas)
    print "DONE"

    print "Finding voronoi points... ",
    mapper.findVoronoiPoints(0.3)
    mapper.drawVoronoiPoints(canvas)
    print "DONE"

    root.mainloop()
                
if __name__ == "__main__":
    run()
