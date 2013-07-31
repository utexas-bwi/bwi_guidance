#!/usr/bin/env python

import rospy

from bwi_msgs.msg import ExperimentServerStatus
from bwi_msgs.srv import UpdateExperimentServer, UpdateExperimentServerResponse

import time
import threading
import random
import string
import subprocess

def id_generator(size=6, chars=string.ascii_uppercase + string.digits):
    return ''.join(random.choice(chars) for x in range(size))

def start_roslaunch_process(package, binary, args={}, log=None):
    command_args = ['roslaunch', package, binary]
    command_args.extend([key + ':=' + value for key, value in args.iteritems()])
    return subprocess.Popen(command_args, stdout=log, stderr=subprocess.STDOUT) if log != None else subprocess.Popen(command_args)

def stop_roslaunch_process(process):
    process.terminate()

class WallRate:

    def __init__(self, rate):
        self.rate = rate
        self.period = 1.0 / rate if rate > 0.0 else 0.0
        self.recorded_time = time.time() 

    def sleep(self):
        current_time = time.time()
        elapsed = current_time - self.recorded_time
        if self.period - elapsed > 0:
            time.sleep(self.period - elapsed)
        self.recorded_time = current_time

class Timer:

    def __init__(self, period = 1.0):
        self.timer_lock = threading.Lock()
        self.reset_vars()
        self.period = period

    def start(self, total_time, callback):
        self.timer_lock.acquire()
        success = False
        if not self.timer:
            self.timer = threading.Timer(self.period, self.tick)
            self.start_time = time.time()
            self.total_time = total_time
            self.time_remaining = total_time
            self.callback = callback
            self.timer.start()
            success = True
        self.timer_lock.release()
        return success

    def tick(self):
        callback = None
        self.timer_lock.acquire()
        if self.timer: # check in case cancel timer has been called
            current_time = time.time() 
            self.time_remaining = self.total_time - (current_time - self.start_time)
            if self.time_remaining <= 0.0:
                callback = self.callback
                self.reset_vars()
            else:
                self.timer = threading.Timer(self.period, self.tick)
                self.timer.start()
        self.timer_lock.release()
        if callback:
            callback()

    def cancel(self):
        self.timer_lock.acquire()
        success = False
        if self.timer:
            self.timer.cancel()
            self.reset_vars()
            success = True
        self.timer_lock.release()
        return success

    def time(self):
        return round(self.time_remaining)

    def reset_vars(self):
        self.total_time = 0.0
        self.time_remaining = 0.0
        self.callback = None
        self.start_time = 0.0
        self.timer = None

class ExperimentServerInterface(threading.Thread):

    def __init__(self, experiment_server_controller):
        threading.Thread.__init__(self)
        self.experiment_status_publisher = rospy.Publisher('~server_status', ExperimentServerStatus)
        self.update_experiment_server = rospy.Service('~update_server', UpdateExperimentServer, self.handleUpdateExperimentRequest)
        self.experiment_controller = experiment_server_controller
        self.interface_lock = threading.Lock()

    def handleUpdateExperimentRequest(self, req):
        resp = UpdateExperimentServerResponse()
        self.interface_lock.acquire()
        if req.lock_experiment:
            resp.result, resp.uid = self.experiment_controller.startExperimentsForNewUser(req.name, req.email)
            if not resp.result:
                resp.status = "The experiment server is already locked. It cannot be locked right now!"
        elif req.unlock_experiment: 
            resp.result = self.experiment_controller.stopExperimentsForUser(req.uid)
            if not resp.result:
                resp.status = "Unable to unlock the experiment server. Do you have a lock on the experiment server?"
        elif req.keep_alive:
            resp.result = self.experiment_controller.keepAliveExperimentsForUser(req.uid)
            if not resp.result:
                resp.status = "Unable to keep the experiment server alive. Do you have a lock on the experiment server?"

        else:
            resp.result = False
            resp.status = "Unable to understand request."
        self.interface_lock.release()
        return resp

    def run(self):
        r = WallRate(20)
        try:
            while not rospy.is_shutdown():
                self.interface_lock.acquire()
                msg = ExperimentServerStatus()
                msg.locked = self.experiment_controller.experiment_server_locked
                msg.uid = self.experiment_controller.experiment_uid
                msg.time_remaining = self.experiment_controller.reset_timer.time()
                self.interface_lock.release()
                self.experiment_status_publisher.publish(msg)
                r.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Server Interface thread shutting down...")

class ExperimentServer:

    def __init__(self):
        rospy.init_node('experiment_server')

        # Get parameters
        self.package = rospy.get_param("~package", "bwi_exp1")
        self.script = rospy.get_param("~script")
        self.logs_folder = rospy.get_param("~logs_folder")
        self.timeout = rospy.get_param("~timeout", 600.0)

        # Server status parameters
        self.experiment_server_locked = False
        self.experiment_uid = ''

        # Setup the experiment text publisher
        self.experiment_interface = ExperimentServerInterface(self)

        # List of current processes
        self.processes = []

        # Prevent race conditions for acquiring a lock on the experiment server
        self.experiment_lock = threading.Lock()

        self.reset_timer = Timer()

    def start(self):
        self.experiment_interface.start()

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Server controller thread shutting down...")

        self.close_all_processes()

    def close_all_processes(self):
        for process in self.processes:
            stop_roslaunch_process(process)
        self.processes = []
            
    def startExperimentsForNewUser(self, name, email):
        self.experiment_lock.acquire()
        success = False
        if not self.experiment_server_locked:
            uid = id_generator()
            rospy.loginfo("Start experiments for " + uid)
            log_file = self.logs_folder + "/" + uid + ".log"
            self.log = open(log_file, "w")
            rospy.loginfo("  Log at: " + log_file)
            process = start_roslaunch_process(self.package, self.script,
                    args={'uid': uid, 'name': name, 'email': email},
                    log=self.log)
            self.processes.append(process)
            self.experiment_server_locked = True
            self.experiment_uid = uid
            self.reset_timer.start(self.timeout, self.resetExperiments)
            success = True
        self.experiment_lock.release()
        return success, uid

    def keepAliveExperimentsForUser(self, uid):
        self.experiment_lock.acquire()
        success = False
        if self.experiment_server_locked and uid == self.experiment_uid:
            self.reset_timer.cancel()
            self.reset_timer.start(self.timeout, self.resetExperiments)
            success = True
        self.experiment_lock.release()
        return success

    def stopExperimentsForUser(self, uid):
        self.experiment_lock.acquire()
        success = False
        if self.experiment_server_locked and uid == self.experiment_uid:
            rospy.loginfo("Stop experiments for " + uid)
            self.log.close()
            self.close_all_processes()
            self.experiment_server_locked = False
            self.experiment_uid = ''
            self.reset_timer.cancel()
            success = True
        self.experiment_lock.release()
        return success

    def resetExperiments(self):
        if self.experiment_server_locked:
            self.stopExperimentsForUser(self.experiment_uid)

if __name__ == '__main__':
    try:
        controller = ExperimentServer()
        controller.start()
    except rospy.ROSInterruptException:
        pass
