#!/usr/bin/env python

import orc_interface

class ListInterface:

    def __init__(self, robots):

        #TODO select only relevant targets here?
        self.robots = robots

    def invoke(self, call_id, target, args):
        pass
        
    def cancel(self, call_id):
        pass

    def check_status(self, call_id):
        ret_message = orc_interface.InterfaceMessage()
        ret_message.type = orc_interface.RETURN
        ret_message.call_id = call_id
        ret_message.value = self.robots
        return True, ret_message
