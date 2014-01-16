#!/usr/bin/env python

import bwi_tools.graph as graph
import csv
import matplotlib.pyplot as plt; plt.rcdefaults()
from matplotlib.font_manager import FontProperties
import sys

MAX_ROBOTS = 5
num_methods = 2
method_names = ['Heuristic', 'VI']

shortest_distances = [44.2129, 55.305, 46.8513, 47.5942, 59.942]
samples = [[[], [], [], [], []], [[], [], [], [], []]]

print 'Reading from: ' + sys.argv[1]
with open(sys.argv[1], 'rb') as csvfile:
    content_reader = csv.reader(csvfile, delimiter=',')
    for line in content_reader:
        if str(line[0]) == 'heuristic':
            for i in range(MAX_ROBOTS):
                print 2 + 2*i
                samples[0][i].append(float(line[2 + 2*i]) / shortest_distances[i])
        else:
            for i in range(MAX_ROBOTS):
                samples[1][i].append(float(line[2 + 2*i]) / shortest_distances[i])

fig, ax, rects, means= \
        graph.draw_bar_chart(samples, method_names, 
                       ('1 Robot', '2 Robots', '3 Robots', '4 Robots', 
                        '5 Robots'),
                       title='Average Normalized Distance',
                       ylabel='Normalized Distance')

sigs = None
if num_methods == 2:
    sigs = []
    for robots in range(MAX_ROBOTS):
        sig = graph.is_significant(samples[0][robots], samples[1][robots])
        if sig:
            print "For " + str(robots + 1) + " robots, diff is significant" 
        else:
            print "For " + str(robots + 1) + " robots, diff is not significant"
        sigs.append(sig)

#attach some text labels
if sigs:
    for r in range(len(rects[0])):
        height = max(rects[0][r].get_height(), rects[1][r].get_height())
        font = FontProperties()
        if sigs[r]:
            font.set_weight('bold')
        ax.text(rects[0][r].get_x()+rects[0][r].get_width(), 1.1*height, 
                '%.3f'%(means[0][r]/means[1][r]),
                ha='center', va='bottom', fontproperties=font)

plt.axhline(y=1.0, xmin=0, xmax=6, linewidth=1, color="black") 
plt.axis([0, 5, 0, 3.0])

fig = plt.gcf()
fig.set_size_inches(6,4)
plt.savefig('out.png',bbox_inches='tight',pad_inches=0.1,dpi=100)

plt.show()
