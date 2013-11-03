#!/usr/bin/env python

import matplotlib.pyplot as plt
import sys
import yaml
import numpy as np

#constants 
gothrough_cost = 5
callforopen_cost = 1
door_cost = gothrough_cost + callforopen_cost
static_approach_cost = 10
goto_cost = 1

# Costs that need to be plotted
paths = []
paths.append([static_approach_cost + 2 * door_cost + goto_cost, 
         (11, 1, 'cor')])
paths.append([static_approach_cost + 4 * door_cost + goto_cost, 
         (12, 13, 'l3_414'),
         (13, 14, 'l3_414a'),
         (14, 1, 'cor')])
paths.append([static_approach_cost + 2 * door_cost + goto_cost, 
         (10, 1, 'cor')])
costs = [[] for p in paths]

file_prefix = sys.argv[1] 
min_iter = 0
if len(sys.argv) > 2:
    min_iter = int(sys.argv[2])
max_iter = 30
if len(sys.argv) > 3:
    max_iter = int(sys.argv[3])

N = max_iter - min_iter + 1
X = np.arange(min_iter, max_iter + 1)
for i in range (min_iter, max_iter + 1):
    with open(file_prefix + str(i), 'r') as yamlfile:
        doc = yaml.load(yamlfile)
        for p in range(len(paths)):
            cost = paths[p][0]
            for action in range(1, len(paths[p])):
                loc = paths[p][action][2]
                from_idx = paths[p][action][0]
                to_idx = paths[p][action][1]
                action_cost_found = False
                for location in doc:
                    if location['name'] == loc:
                        for yaml_costs in location['costs']:
                            if from_idx == yaml_costs['from'] and to_idx == yaml_costs['to']:
                                cost += yaml_costs['cost']
                                action_cost_found = True
                                break
                    if action_cost_found:
                        break
                if not action_cost_found:
                    print "action cost not found!"
            costs[p].append(cost)

fig, ax = plt.subplots()
p1, = ax.plot(X, np.array(costs[0]), linewidth = 4)
p2, = ax.plot(X, np.array(costs[1]), dashes=(20,5), linewidth = 4)
p3, = ax.plot(X, np.array(costs[2]), dashes=(15,5,5,5), linewidth = 4)

ax.set_ylabel('Path Cost Estimate')
ax.set_xlabel('Episode Number')
ax.set_title('Cost Learning')
 
ax.legend((p1,p2,p3), ('Path 1', 'Path 2', 'Path 3'), handlelength=4)
plt.axis([min_iter, max_iter, 20, 180])
#plt.show()
 
fig = plt.gcf()
fig.set_size_inches(6,4)
plt.savefig('out.png',bbox_inches='tight',pad_inches=0.1,dpi=300)
