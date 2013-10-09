#!/usr/bin/env python

import csv
import matplotlib.pyplot as plt; plt.rcdefaults()
import numpy as np
import scipy.stats as stats
import sys

def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), stats.sem(a)
    h = se * stats.t._ppf((1+confidence)/2., n-1)
    return m, h

hs = [[], [], [], [], []]
hs_vals = [[], [], [], [], []]
vi = [[], [], [], [], []] 

print 'Reading from: ' + sys.argv[1]
with open(sys.argv[1], 'rb') as csvfile:
    content_reader = csv.reader(csvfile, delimiter=',')
    for line in content_reader:
        for i in range(5):
            vi[i].append(float(line[4 + 3*i]) / float(line[18]))
            hs[i].append(float(line[5 + 3*i]) / float(line[18]))
            if float(line[4 + 3*i]) > 20.0 and float(line[5 + 3*i]) > 20.0:
                hs_vals[i].append([vi[i][-1], [line[1], line[2], line[3]]])

for i in range(5):
    hs_vals[i].sort(reverse=True, key=lambda x: x[0])
    print hs_vals[i][0:15]

vi_means = []
vi_conf = []

hs_means = []
hs_conf = []

for i in range(5):
    m, h = mean_confidence_interval(vi[i])
    vi_means.append(m)
    vi_conf.append(h)
    m, h = mean_confidence_interval(hs[i])
    hs_means.append(m)
    hs_conf.append(h)

ind = np.arange(5)
width = 0.35
fig, ax = plt.subplots()
rects1 = ax.bar(ind, hs_means, width, color='y', yerr=hs_conf)
rects2 = ax.bar(ind+width, vi_means, width, color='r', yerr=vi_conf)
ax.set_ylabel('Normalized Distance')
ax.set_title('Average Normalized Distance - ' + sys.argv[2])
ax.set_xticks(ind+width)
ax.set_xticklabels( ('1 Robot', '2 Robots', '3 Robots', '4 Robots', '5 Robots') )

ax.legend( (rects1[0], rects2[0]), ('Heuristic', 'VI') )

# attach some text labels
for r in range(len(rects1)):
    height = rects1[r].get_height()
    ax.text(rects1[r].get_x()+width, 1.1*height, '%.3f'%(hs_means[r]/vi_means[r]),
            ha='center', va='bottom')

plt.axhline(y=1.0, xmin=0, xmax=6, linewidth=1, color="black") 
plt.axis([0, 5, 0, 6])
# plt.show()

fig = plt.gcf()
fig.set_size_inches(6,4)
plt.savefig('out.png',bbox_inches='tight',pad_inches=0.1,dpi=100)
