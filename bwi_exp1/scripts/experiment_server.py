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

def start_roslaunch_process(package, binary, args={}):
    command_args = ['roslaunch', package, binary]
    command_args.extend([key + ':=' + value for key, value in args.iteritems()])
    return subprocess.Popen(command_args)

def stop_roslaunch_process(process):
    process.terminate()

class ExperimentServerInterface:

    def __init__(self, experiment_server_controller):
        self.experiment_status_publisher = rospy.Publisher('~server_status', ExperimentServerStatus)
        self.update_experiment_server = rospy.Service('~update_server', UpdateExperimentServer, self.handleUpdateExperimentRequest)
        self.experiment_controller = experiment_server_controller

    def handleUpdateExperimentRequest(self, req):
        resp = UpdateExperimentServerResponse()
        if req.lock_experiment:
            resp.result, resp.uid = self.experiment_controller.startExperimentsForNewUser(req.name, req.email)
            if not resp.result:
                resp.status = "The experiment server is already locked. It cannot be locked right now!"
        elif req.unlock_experiment: 
            resp.result = self.experiment_controller.stopExperimentsForUser(req.uid)
            if not resp.result:
                resp.status = "Unable to unlock the experiment server. Do you have a lock on the experiment server?"
        else:
            resp.result = False
            resp.status = "Unable to understand request."
        return resp

    def publishStatus(self):
        #TODO this should publish on its own
        msg = ExperimentServerStatus()
        msg.locked = self.experiment_controller.experiment_server_locked
        msg.uid = self.experiment_controller.experiment_uid
        self.experiment_status_publisher.publish(msg)

class ExperimentServerController:

    def __init__(self):
        rospy.init_node('experiment_server')

        # Get parameters
        self.environment_script = rospy.get_param("~environment")
        self.controller_script = rospy.get_param("~controller")

        # Server status parameters
        self.experiment_server_locked = False
        self.experiment_uid = ''

        # Setup the experiment text publisher
        self.experiment_interface = ExperimentServerInterface(self)

        # List of current processes
        self.processes = []

        # Prevent race conditions for acquiring a lock on the experiment server
        self.experiment_lock = threading.Lock()

    def start(self):

        rate = 10.0 # 10hz
        period = 1/rate

        while not rospy.is_shutdown():
            now = time.time()
            self.experiment_lock.acquire()
            self.experiment_interface.publishStatus()
            elapsed = time.time() - now
            if period - elapsed > 0:
                time.sleep(period - elapsed)
            self.experiment_lock.release()

    def close_all_processes(self):
        for process in self.processes:
            stop_roslaunch_process(process)
        self.processes = []
            
    def startExperimentsForNewUser(self, name, email):
        self.experiment_lock.acquire()
        success = False
        if not self.experiment_server_locked:
            uid = id_generator()
            con = start_roslaunch_process('bwi_exp1', self.controller_script,
                    args={'uid': uid, 'name': name, 'email': email})
            self.processes.append(con)
            env = start_roslaunch_process('bwi_exp1', self.environment_script)
            self.processes.append(env)
            self.experiment_server_locked = True
            self.experiment_uid = uid
            success = True
        self.experiment_lock.release()
        return success, uid

    def stopExperimentsForUser(self, uid):
        self.experiment_lock.acquire()
        success = False
        if self.experiment_server_locked and uid == self.experiment_uid:
            self.close_all_processes()
            self.experiment_server_locked = False
            self.experiment_uid = False
            success = True
        self.experiment_lock.release()
        return success

if __name__ == '__main__':
    try:
        controller = ExperimentServerController()
        controller.start()
    except rospy.ROSInterruptException:
        pass
