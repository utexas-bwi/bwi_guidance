#!/usr/bin/env python

import bwi_tools.graph as graph
import csv
import simplejson as json
import matplotlib.pyplot as plt; plt.rcdefaults()
from matplotlib.font_manager import FontProperties
import pylab
import sys

num_methods = 5
MAX_ROBOTS = 5
samples = []
for i in range(num_methods):
    samples.append([])
    for j in range(MAX_ROBOTS):
        samples[i].append([])

first = True
print 'Reading reward from: ' + sys.argv[1]
with open(sys.argv[1], 'rb') as csvfile:
    content_reader = csv.reader(csvfile, delimiter=',')
    for line in content_reader:
        for i in range(num_methods):
            for j in range(MAX_ROBOTS):
                samples[i][j].append(-float(line[i + num_methods*j]))

top_level_names = ['10','20','30','40','50']
second_level_names = ['1','2','3','4','5']
third_level_names = ['0.0','-0.5','-1','-1.5','-2','-2.5', '-3', '-3.5', '-4']
fig, ax, rects, means= \
        graph.draw_3d_bar_chart(samples, top_level_names=top_level_names,
                             second_level_names=second_level_names,
                                third_level_names=third_level_names,
                       xlabel='visRange(meters)', 
                       ylabel='adjDepth', 
                       zlabel='Normalized Reward (Negated)', 
                                flip_y=False)

ax.view_init(15,-135)

fig = plt.gcf()
fig.set_size_inches(6,6)
plt.savefig('out.png',bbox_inches='tight',pad_inches=0.1,dpi=300)

plt.show()

# pylab.figure()
# for method in range(num_methods): 
#     pylab.hist(samples[method][4], [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 20.0, 50.0, 100.0], histtype='step', label=method_names[method])
#     #pylab.hist(samples[method][4], 100, histtype='step', label=method_names[method])
# pylab.yscale('log')
# pylab.xscale('log')
# pylab.legend()
# pylab.show()
