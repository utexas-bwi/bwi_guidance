#!/usr/bin/env python

import select
import signal
import socket
import threading

import bwi_exp1
import comms

class Server:

    def __init__(self, callback=None, host='', port=12345, max_connections=1):

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host,port))
        print 'Listening to port',port,'...'
        self.server.listen(max_connections)

        self.outputs = []
        self.inputs = [self.server]

        signal.signal(signal.SIGINT, self.signal_handler)
        self.callback = callback

    def start(self):

        rate = bwi_exp1.WallRate(100)

        while True:
            try:
                inputready,outputready,exceptready = \
                        select.select(self.inputs, self.outputs, [])
            except select.error, e:
                break
            except socket.error, e:
                break

            for s in inputready:
                if s == self.server:
                    # handle the server socket
                    client, address = self.server.accept()
                    print 'chatserver: got connection %d from %s' % (client.fileno(), address)
                    self.inputs.append(client)
                else:
                    try:
                        type, data = comms.recv(s)
                        if type == comms.ENQ:
                            comms.send(s, comms.ACK)
                        elif type == comms.EOT:
                            print 'server: removing connection on EOT request'
                            self.inputs.remove(s)
                            self.outputs.remove(s)
                        elif type == comms.MESSAGE:
                            success = False
                            if data:
                                success = self.callback(self.get_name(s), data)
                            if success:
                                comms.send(s, comms.ACK)
                            else:
                                comms.send(s, comms.NACK)
                        elif type == comms.ACK or type == comms.NACK:
                            self.response_received[self.get_name(s)] = type 
                                
                    except socket.error, e:
                        print 'server: removing connection on socket exception'
                        comms.send(s, comms.EOT)
                        self.inputs.remove(s)
                        self.outputs.remove(s)
            rate.sleep()

    def send_message(self, client_name, data):
        client = self.get_client(client_name)
        if client == None:
            return False #TODO throw exception here
        self.response_received[client_name] = None
        comms.send(self.get_client(client_name))
        timer = threading.Timer(30.0, self.send_message_failed, [client_name])
        timer.start()
        rate = bwi_exp1.WallRate(10)
        while self.response_received == None:
            rate.sleep()
        timer.cancel()
        if self.response_received == comms.ACK:
            return True
        return False

    def send_message_failed(self, client_name):
        self.response_received[client_name] = comms.NACK #TODO throw exception here

    def signal_handler(self, signum, frame): 
        # Close existing client sockets
        print 'Shutting down server...'
        for o in self.outputs:
            o.close()
        self.server.close()




