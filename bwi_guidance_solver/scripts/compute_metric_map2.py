#!/usr/bin/env python

import csv
import numpy as np
import scipy.stats as stats
import sys

def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), stats.sem(a)
    h = se * stats.t._ppf((1+confidence)/2., n-1)
    return m, h

hs = [[], [], [], [], [], [], [], [], [], [], [], [], [], [], []]

print 'Reading from: ' + sys.argv[1]
with open(sys.argv[1], 'rb') as csvfile:
    content_reader = csv.reader(csvfile, delimiter=',')
    for line in content_reader:
        for i in range(15):
            hs[i].append(float(line[5 + i]) / float(line[4]))

hs_means = []
hs_conf = []

for i in range(15):
    m, h = mean_confidence_interval(hs[i])
    hs_means.append(m)
    hs_conf.append(h)

print hs_means
print hs_conf
