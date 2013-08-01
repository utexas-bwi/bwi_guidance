#!/usr/bin/env python

import rospy
import socket
import select
import json
import StringIO

START_TEXT = unichr(2)
MESSAGE = START_TEXT
END_TEXT = unichr(3)
ACK = unichr(6)
NACK = unichr(15)
ENQUIRY = unichr(5)
EOT = unichr(4)

def send(channel, type, object=None):
    if type == MESSAGE:
        object_str = json.dumps(object)
        frame_str = START_TEXT + object_str + END_TEXT
        channel.send(frame_str)
    else:
        channel.send(type)

def receive(channel):
    byte = channel.recv(1)
    if len(byte) == 0:
        return None, None
    if byte == START_TEXT:
        buf = ''
        while True:
            byte = channel.recv(1)
            if byte == END_TEXT:
                break
            buf = buf + byte
        return MESSAGE, json.loads(buf)

    else:
        return byte, None

class OrcInterface:

    def __init__(self, host='', port=12345, backlog=5):
        rospy.init_node('experiment_server')
        self.outputs = []
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host,port))
        rospy.loginfo('Listening to port: ' + str(port))
        self.server.listen(backlog)

    def start(self):
        while not rospy.is_shutdown():
            try:
                inputready,outputready,exceptready = \
                        select.select([self.server], self.outputs, [])
        
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        rospy.loginfo("Server thread shutting down...")


if __name__ == '__main__':
    try:
        server = OrcInterface()
        server.start()
    except rospy.ROSInterruptException:
        pass
