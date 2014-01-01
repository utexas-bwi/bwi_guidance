#!/usr/bin/env python

import csv
import matplotlib.pyplot as plt; plt.rcdefaults()
from matplotlib.font_manager import FontProperties
import numpy as np
import scipy.stats as stats
import sys

# Method names corresponding to method types
METHOD_NAMES = ['Heuristic', 'VI', 'UCT']
MAX_ROBOTS = 5 # Compute this from the first line you encounter

METHOD_COLORS = ['y', 'r', 'lightblue']
METHOD_HATCH = ['/', '\\', 'x']

"""
import json
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
            if key == "gamma":
                name += "gamma=" + str(value) # TODO get symbol for gamma, formatted float value
            elif key == "use_intrinsic_reward":
                name += "IntrinsicReward"
            elif key == "success_reward":
                name += "SuccessReward=" + str(value) # This parameter needs to be names
            elif key == "mtcs_reward_bound":
                name += "C=" + str(value) # Might change if parameter changes
            elif key == "mtcs_initial_planning_time":
                name += "InitialPlanningTime=" + str(value) + "s"

            parameter_count += 1
            if parameter_count != num_parameters:
                name += ","
        name += "]"
    method_names.append(name)

"""
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

num_methods = 3 
samples = []
means = []
confs = []
for i in range(num_methods):
    samples.append([])
    means.append([])
    confs.append([])
    for j in range(MAX_ROBOTS):
        samples[i].append([])

first = True
print 'Reading from: ' + sys.argv[1]
with open(sys.argv[1], 'rb') as csvfile:
    content_reader = csv.reader(csvfile, delimiter=',')
    for line in content_reader:
        if first:
            num_methods = len(line) / MAX_ROBOTS
            first = False
        for i in range(num_methods):
            for j in range(MAX_ROBOTS):
                samples[i][j].append(float(line[i + num_methods*j]))


for i in range(num_methods):
    for j in range(MAX_ROBOTS):
        m, h = mean_standard_error(samples[i][j])
        means[i].append(m)
        confs[i].append(h)

# sigs = []
#     sig = is_significant(vi[i], hs[i])
#     if sig:
#         print "For " + str(i + 1) + " robots, diff is significant" 
#     else:
#         print "For " + str(i + 1) + " robots, diff is not significant"
#     sigs.append(sig)

ind = np.arange(5)
width = 1.0 / (num_methods + 1)
fig, ax = plt.subplots()
rects = []
for i in range(num_methods):
    rect = ax.bar(ind + i*width, means[i], width, color=METHOD_COLORS[i], hatch=METHOD_HATCH[i], yerr=confs[i])
    rects.append(rect)
ax.set_ylabel('Normalized Distance')
ax.set_title('Average Normalized Distance - ' + sys.argv[2])
ax.set_xticks(ind+width)
ax.set_xticklabels( ('1 Robot', '2 Robots', '3 Robots', '4 Robots', '5 Robots') )

ax.legend(rects, METHOD_NAMES[:len(rects)])

# attach some text labels
# for r in range(len(rects1)):
#     height = max(rects1[r].get_height(), rects2[r].get_height())
#     font = FontProperties()
#     if sigs[r]:
#         font.set_weight('bold')
#     ax.text(rects1[r].get_x()+width, 1.1*height, '%.3f'%(hs_means[r]/vi_means[r]),
#             ha='center', va='bottom', fontproperties=font)

plt.axhline(y=1.0, xmin=0, xmax=6, linewidth=1, color="black") 
plt.axis([0, 5, 0, 6])
plt.show()

# fig = plt.gcf()
# fig.set_size_inches(6,4)
# plt.savefig('out.png',bbox_inches='tight',pad_inches=0.1,dpi=100)
