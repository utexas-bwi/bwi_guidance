#!/usr/bin/env python

import bwi_tools.graph as graph
import csv
import simplejson as json
import matplotlib.pyplot as plt; plt.rcdefaults()
from matplotlib.font_manager import FontProperties
import pylab
import sys

num_methods = 2
MAX_ROBOTS = 1
samples = []
for i in range(num_methods):
    samples.append([])
    for j in range(MAX_ROBOTS):
        samples[i].append([])

first = True
print 'Reading with statemerge from: ' + sys.argv[1]
print 'Reading without statemerge from: ' + sys.argv[2]
with open(sys.argv[1], 'rb') as csvfile:
    content_reader = csv.reader(csvfile, delimiter=',')
    for line in content_reader:
        for j in range(MAX_ROBOTS):
            samples[0][j].append(float(line[num_methods*j]))
with open(sys.argv[2], 'rb') as csvfile:
    content_reader = csv.reader(csvfile, delimiter=',')
    for line in content_reader:
        for j in range(MAX_ROBOTS):
            samples[1][j].append(float(line[num_methods*j]))

fig, ax, rects, means= \
        graph.draw_bar_chart(samples, ["UCT", "UCT-NSM"],
                             second_level_names=('',),
                       ylabel='Normalized Reward')

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
        ax.text(rects[0][r].get_x()+rects[0][r].get_width(), -1.2*height, 
                '%.3f'%(means[0][r]/means[1][r]),
                ha='center', va='bottom', fontproperties=font)

plt.axhline(y=0.0, xmin=0, xmax=2, linewidth=1, color="black") 
plt.axis([-rects[0][r].get_width(), 3*rects[0][r].get_width(), -5, 2])

fig = plt.gcf()
fig.set_size_inches(2.3,4)
plt.savefig('out.png',bbox_inches='tight',pad_inches=0.1,dpi=150)

plt.show()

# pylab.figure()
# for method in range(num_methods): 
#     pylab.hist(samples[method][4], [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 20.0, 50.0, 100.0], histtype='step', label=method_names[method])
#     #pylab.hist(samples[method][4], 100, histtype='step', label=method_names[method])
# pylab.yscale('log')
# pylab.xscale('log')
# pylab.legend()
# pylab.show()
