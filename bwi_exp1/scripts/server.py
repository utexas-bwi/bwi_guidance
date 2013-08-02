#!/usr/bin/env python

import bwi_exp1
import bwi_tools
import random
import roslib
import rospy
import threading

from bwi_msgs.msg import ExperimentServerStatus
from bwi_msgs.srv import UpdateExperimentServer, UpdateExperimentServerResponse
from string import ascii_uppercase, digits

def id_generator(size=6, chars=ascii_uppercase + digits):
    return ''.join(random.choice(chars) for x in range(size))

class ExperimentServerInterface(threading.Thread):

    def __init__(self, server):
        threading.Thread.__init__(self)
        self.server_status_publisher = rospy.Publisher(
                '~server_status', ExperimentServerStatus)
        self.update_experiment_server = rospy.Service(
                '~update_server', UpdateExperimentServer, 
                self.handle_server_update)
        self.server = server
        self.interface_lock = threading.Lock()

    def handle_server_update(self, req):
        resp = UpdateExperimentServerResponse()
        self.interface_lock.acquire()
        if req.lock_experiment:
            resp.result, resp.uid = \
                    self.server.start_experiments(
                        req.name, req.email)
            if not resp.result:
                resp.status = "The experiment server is already locked. " \
                        "It cannot be locked right now!"
        elif req.unlock_experiment: 
            resp.result = \
                    self.server.stop_experiments(req.uid)
            if not resp.result:
                resp.status = "Unable to unlock the experiment server. " \
                        "Do you have a lock on the experiment server?"
        elif req.keep_alive:
            resp.result = \
                    self.server.keep_user_alive(req.uid)
            if not resp.result:
                resp.status = "Unable to keep the experiment server alive. " \
                        "Do you have a lock on the experiment server?"
        else:
            resp.result = False
            resp.status = "Unable to understand request."
        self.interface_lock.release()
        return resp

    def run(self):
        rate = bwi_tools.WallRate(10)
        try:
            while not rospy.is_shutdown():
                self.interface_lock.acquire()
                msg = ExperimentServerStatus()
                msg.locked = self.server.experiment_server_locked
                msg.uid = self.server.experiment_uid
                msg.time_remaining = self.server.reset_timer.time()
                self.interface_lock.release()
                self.server_status_publisher.publish(msg)
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        rospy.loginfo("Server Interface thread shutting down...")

class ExperimentServer:

    def __init__(self):
        rospy.init_node('experiment_server')

        # Get parameters
        self.package = rospy.get_param("~package", "bwi_exp1")
        self.script = rospy.get_param("~script")
        self.logs_folder = rospy.get_param("~logs_folder",
                 roslib.packages.get_pkg_dir("bwi_exp1") + "/logs")
        self.timeout = rospy.get_param("~timeout", 600.0)

        # Server status parameters
        self.experiment_server_locked = False
        self.experiment_uid = ''
        self.log = None

        # Setup the experiment text publisher
        self.experiment_interface = ExperimentServerInterface(self)

        # List of current processes
        self.processes = []

        # Prevent race conditions for acquiring a lock on the experiment server
        self.experiment_lock = threading.Lock()

        self.reset_timer = bwi_tools.Timer()

    def start(self):
        self.experiment_interface.start()

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        rospy.loginfo("Server thread shutting down...")

        self.close_all_processes()

    def close_all_processes(self):
        for process in self.processes:
            bwi_exp1.stop_roslaunch_process(process)
        self.processes = []
            
    def start_experiments(self, name, email):
        self.experiment_lock.acquire()
        success = False
        if not self.experiment_server_locked:
            uid = id_generator()
            rospy.loginfo("Start experiments for " + uid)
            log_file = self.logs_folder + "/" + uid + ".log"
            self.log = open(log_file, "w")
            rospy.loginfo("  Log at: " + log_file)
            process = bwi_exp1.start_roslaunch_process(
                    self.package, self.script,
                    args={'uid': uid, 'name': name, 'email': email},
                    log=self.log)
            self.processes.append(process)
            self.experiment_server_locked = True
            self.experiment_uid = uid
            self.reset_timer.start(self.timeout, self.reset_server)
            success = True
        self.experiment_lock.release()
        return success, uid

    def keep_user_alive(self, uid):
        """Restarts the reset timer with the max timeout duration. This function
        implements a keep alive ping that is used to prevent a user from holding
        the server for arbitrarily long.
        
        """
        self.experiment_lock.acquire()
        success = False
        if self.experiment_server_locked and uid == self.experiment_uid:
            self.reset_timer.cancel()
            self.reset_timer.start(self.timeout, self.reset_server)
            success = True
        self.experiment_lock.release()
        return success

    def stop_experiments(self, uid):
        """Stops a running experiment if provided uid matches the current one"""
        self.experiment_lock.acquire()
        success = False
        if self.experiment_server_locked and uid == self.experiment_uid:
            rospy.loginfo("Stop experiments for " + uid)
            self.log.close()
            self.log = None
            self.close_all_processes()
            self.experiment_server_locked = False
            self.experiment_uid = ''
            self.reset_timer.cancel()
            success = True
        self.experiment_lock.release()
        return success

    def reset_server(self):
        if self.experiment_server_locked:
            self.stop_experiments(self.experiment_uid)

if __name__ == '__main__':
    try:
        server = ExperimentServer()
        server.start()
    except rospy.ROSInterruptException:
        pass
