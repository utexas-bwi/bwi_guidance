#!/usr/bin/env python

import os
import sys

data_directory = '/projects/agents1/piyushk/map3/'
if len(sys.argv) > 1:
    data_directory = sys.argv[1]
    print "Processing directory: " + data_directory

max_results = 1000
if len(sys.argv) > 2:
    max_results = int(sys.argv[2])

suffixes = ['time.txt','utility.txt','reward.txt']
for suffix in suffixes:
    outfile = open('iros_' + suffix, "w")
    for i in range(max_results):
        cur_input_file = data_directory + str(i) + '_' + suffix
        if os.path.exists(cur_input_file):
            print "Processing " + cur_input_file
            infile = open(cur_input_file,  "r")
            line = infile.readline()
            if line:
                outfile.write(line)
            else:
                print "File " + cur_input_file + "exists, but no data"
                infile.close()

    outfile.close()
