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
#A-D-H
paths.append([static_approach_cost + 6 * door_cost + 3 * goto_cost, 
         (0,1, 'cor'), (1,14,'cor'), (14,15,'l3_414a'), (15,20,'cor')])
#A-H-D
paths.append([static_approach_cost + 6 * door_cost + 3 * goto_cost, 
         (0,1, 'cor'), (1,20,'cor'), (20,15,'cor')])
#D-A-H
paths.append([static_approach_cost + 6 * door_cost + 3 * goto_cost, 
         (0,14, 'cor'), (14,1,'cor'), (1,20,'cor')])
#D-H-A
paths.append([static_approach_cost + 6 * door_cost + 3 * goto_cost, 
         (0,14, 'cor'), (15,20,'cor'), (20,1,'cor')])
#H-A-D
paths.append([static_approach_cost + 6 * door_cost + 3 * goto_cost, 
         (0,20, 'cor'), (20,1,'cor'), (1,14,'cor')])
#H-D-A
paths.append([static_approach_cost + 6 * door_cost + 3 * goto_cost, 
         (0,20, 'cor'), (20,15,'cor'), (15,14,'l3_414a'), (14,1,'cor')])
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
p1, = ax.plot(X, np.array(costs[0]), linewidth = 3)
p2, = ax.plot(X, np.array(costs[1]), dashes=(20,5), linewidth = 3)
p3, = ax.plot(X, np.array(costs[2]), dashes=(15,5,5,5), linewidth = 3)
p4, = ax.plot(X, np.array(costs[3]), linewidth = 3)
p5, = ax.plot(X, np.array(costs[4]), dashes=(20,5), linewidth = 3, color='lightgray')
p6, = ax.plot(X, np.array(costs[5]), dashes=(15,5,5,5), linewidth = 3)

ax.set_ylabel('Path Cost Estimate')
ax.set_xlabel('Episode Number')
ax.set_title('Cost Learning')
 
ax.legend((p1,p2,p3,p4,p5,p6), ('A-D-H', 'A-H-D', 'D-A-H', 'D-H-A', 'H-A-D', 'H-D-A'), handlelength=4, mode='expand', ncol=3)
#ax.legend([p1], ['A-D-H'], handlelength=4)
plt.axis([min_iter, max_iter, 45, 180])
#plt.show()
 
fig = plt.gcf()
fig.set_size_inches(7,4)
plt.savefig('out.png',bbox_inches='tight',pad_inches=0.1,dpi=300)
