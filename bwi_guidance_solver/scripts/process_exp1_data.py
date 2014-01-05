#!/usr/bin/env python

import collections
import csv
import json
import matplotlib.pyplot as plt; plt.rcdefaults()
from matplotlib.font_manager import FontProperties
import numpy as np
import scipy.stats as stats
import sys

# Method names corresponding to method types
METHOD_NAMES = ['Heuristic', 'VI', 'UCT']

# Keep the following at different length to produce more distinct combinations
METHOD_COLORS = ['yellow', 'red', 'aqua', 'green', 'lightgray', 'blue']
METHOD_HATCH = ['/', '\\', 'x', '*', 'o', 'O', '.']

def mean_standard_error(data):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), stats.sem(a)
    return m, se

def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), stats.sem(a)
    h = se * stats.t._ppf((1+confidence)/2., n-1)
    return m, h

def is_significant(a, b, confidence=0.95):
    t, p = stats.ttest_ind(a, b)
    return p < (1.0 - confidence)

def draw_bar_chart(samples, top_level_names, second_level_names=None, 
                  title=None, xlabel=None, ylabel=None):

    # So, samples can either contain a list of lists. The top level list
    # contains top level groups, and the second level list contains actual
    # samples (top_level_grouping_only = true)

    # Alternatively, samples may be a list of lists of lists, with top-level 
    # groups, second level groups and actual samples. (top_level_grouping_only)

    means = []
    confs = []
    second_level_grouping_available = \
            isinstance(samples[0][0], collections.Sequence)
    top_level_methods = len(samples)

    if second_level_grouping_available: 
        second_level_methods = len(samples[0])
        samples2 = samples
    else:
        # Create artificial second level grouping
        second_level_methods = 1
        samples2 = [[samples[i]] for i in range(top_level_methods)]

    for i in range(top_level_methods):
        means.append([])
        confs.append([])

    for i in range(top_level_methods):
        for j in range(second_level_methods):
            m, h = mean_standard_error(samples[i][j])
            means[i].append(m)
            confs[i].append(h)

    ind = np.arange(second_level_methods)
    width = 1.0 / (top_level_methods + 1)
    fig, ax = plt.subplots()
    rects = []
    for i in range(top_level_methods):
        rect = ax.bar(ind + i*width, means[i], width,
                      color=METHOD_COLORS[i % len(METHOD_COLORS)], 
                      hatch=METHOD_HATCH[i % len(METHOD_HATCH)],
                      yerr=confs[i])
        rects.append(rect)

    if xlabel:
        ax.set_ylabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)

    if second_level_grouping_available:
        ax.set_xticks(ind+0.5-width/2)
        if second_level_names:
            ax.set_xticklabels(second_level_names)

    ax.legend(rects, top_level_names)

    return fig, ax, rects

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
            elif key == "use_intrinsic_reward":
                param = "IntrinsicReward"
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

fig, ax, rects = \
        draw_bar_chart(samples, method_names, 
                       ('1 Robot', '2 Robots', '3 Robots', '4 Robots', 
                        '5 Robots'),
                       title='Average Normalized Distance',
                       ylabel='Normalized Distance')

# sigs = []
#     sig = is_significant(vi[i], hs[i])
#     if sig:
#         print "For " + str(i + 1) + " robots, diff is significant" 
#     else:
#         print "For " + str(i + 1) + " robots, diff is not significant"
#     sigs.append(sig)

# attach some text labels
# for r in range(len(rects1)):
#     height = max(rects1[r].get_height(), rects2[r].get_height())
#     font = FontProperties()
#     if sigs[r]:
#         font.set_weight('bold')
#     ax.text(rects1[r].get_x()+width, 1.1*height, '%.3f'%(hs_means[r]/vi_means[r]),
#             ha='center', va='bottom', fontproperties=font)

plt.axhline(y=1.0, xmin=0, xmax=6, linewidth=1, color="black") 
plt.axis([0, 5, 0, 7])
plt.show()

# fig = plt.gcf()
# fig.set_size_inches(6,4)
# plt.savefig('out.png',bbox_inches='tight',pad_inches=0.1,dpi=100)
