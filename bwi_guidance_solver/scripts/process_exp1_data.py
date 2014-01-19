#!/usr/bin/env python

import bwi_tools.graph as graph
import csv
import simplejson as json
import matplotlib.pyplot as plt; plt.rcdefaults()
from matplotlib.font_manager import FontProperties
import pylab
import sys

# Method names corresponding to method types
METHOD_NAMES = ['Heuristic', 'VI', 'UCT']

MAX_ROBOTS = 5 # Compute this from the first line you encounter
config_file_name = sys.argv[1]
print 'Reading from: ' + config_file_name
config_file = open(config_file_name, "r")
methods = json.load(config_file)
num_methods = len(methods['methods'])
method_names = []
for method in methods['methods']:
    name = METHOD_NAMES[method['type']]
    num_parameters = len(method) - 1 # type is not a parameter
    if len(method) > 1:
        name += "["
        parameter_count = 0
        for key,value in method.iteritems():
            param = None
            if key == "gamma":
                param = "gamma=" + str(value) # TODO get symbol for gamma, formatted float value
            elif key == "reward_structure" and value == 0:
                param = "StandardReward"
            elif key == "reward_structure" and value == 1:
                param = "IntrinsicReward"
            elif key == "reward_structure" and value == 2:
                param = "ShapingReward"
            elif key == "success_reward":
                param = "EndReward=" + str(value) # This parameter needs to be names
            elif key == "mcts_reward_bound":
                param = "RewardBound=" + str(value) # Might change if parameter changes
            elif key == "mcts_initial_planning_time":
                param = "InitialPlanningTime=" + str(value) + "s"
            
            if param and parameter_count != 0:
                name += ","
            if param:
                name += param
                parameter_count += 1

        name += "]"
    method_names.append(name)

samples = []
for i in range(num_methods):
    samples.append([])
    for j in range(MAX_ROBOTS):
        samples[i].append([])

first = True
print 'Reading from: ' + sys.argv[2]
with open(sys.argv[2], 'rb') as csvfile:
    content_reader = csv.reader(csvfile, delimiter=',')
    for line in content_reader:
        for i in range(num_methods):
            for j in range(MAX_ROBOTS):
                samples[i][j].append(float(line[i + num_methods*j]))

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
plt.axis([0, 5, 0, 7])

fig = plt.gcf()
fig.set_size_inches(6,4)
plt.savefig('out.png',bbox_inches='tight',pad_inches=0.1,dpi=100)

plt.show()

pylab.figure()
for method in range(num_methods): 
    pylab.hist(samples[method][4], [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 20.0, 50.0, 100.0], histtype='step', label=method_names[method])
    #pylab.hist(samples[method][4], 100, histtype='step', label=method_names[method])
pylab.yscale('log')
pylab.xscale('log')
pylab.legend()
pylab.show()
