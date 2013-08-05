#!/usr/bin/env python

import subprocess

def start_roslaunch_process(package, binary, args=None, log=None):
    if args == None:
        args = {}
    command_args = ['roslaunch', package, binary]
    command_args.extend([key + ':=' + value for key, value in args.iteritems()])
    print "Running command: " + ' '.join(command_args)
    return (subprocess.Popen(command_args, stdout=log, stderr=subprocess.STDOUT)
            if log != None else subprocess.Popen(command_args))

def stop_roslaunch_process(process):
    process.terminate()


