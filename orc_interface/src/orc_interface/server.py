#!/usr/bin/env python

import select
import socket
import threading

import bwi_tools
import comms

class ClientNotFoundException(Exception):
        pass

class Server(threading.Thread):

    def __init__(self, callback=None, host='', port=12345, max_connections=1):
        
        threading.Thread.__init__(self)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host,port))
        print 'Server: listening to port',port,'...'
        self.server.listen(max_connections)

        self.inputs = [self.server]

        #signal.signal(signal.SIGINT, self.signal_handler)
        self.callback = callback

        self.client_map = {}
        self.client_name_map = {}
        self.response_received = {}

        self.alive = False
        self.server_lock = threading.Lock()

    def run(self):

        rate = bwi_tools.WallRate(20)

        self.alive = True
        while True:
            self.server_lock.acquire()
            if not self.alive:
                break
            try:
                inputready,outputready,exceptready = \
                        select.select(self.inputs, [], [], 0.01)
            except select.error, e:
                break
            except socket.error, e:
                break

            for s in inputready:
                if s == self.server:
                    # handle the server socket
                    client, address = self.server.accept()
                    self.add_client(client, address)
                else:
                    try:
                        type, data = comms.recv(s)
                        if type == comms.ENQ:
                            comms.send(s, comms.ACK)
                        elif type == comms.EOT:
                            self.remove_client(s)
                        elif type == comms.MSG:
                            success = False
                            if data and self.callback != None:
                                success = self.callback(self.get_client_name(s), data)
                            if success:
                                comms.send(s, comms.ACK)
                            else:
                                comms.send(s, comms.NACK)
                        elif type == comms.ACK or type == comms.NACK:
                            self.response_received[self.get_client_name(s)] = type 
                                
                    except socket.error, e:
                        self.remove_client(s)

            self.server_lock.release()
            rate.sleep()

    def send_message(self, client_name, data):
        client = self.get_client(client_name)
        if client == None:
            raise ClientNotFoundException()
        self.response_received[client_name] = None
        comms.send(client, comms.MSG, data)
        timer = threading.Timer(30.0, self.send_message_failed, [client_name])
        timer.start()
        rate = bwi_tools.WallRate(10)
        while self.response_received == None:
            rate.sleep()
        timer.cancel()
        if self.response_received == comms.ACK:
            return True
        return False

    def send_message_failed(self, client_name):
        self.response_received[client_name] = comms.NACK

    def generate_client_name(self, address):
        return str(address[1]) + '@' + str(address[0])

    def get_client(self, client_name):
        if client_name in self.client_map:
            return self.client_map[client_name]
        return None

    def get_client_name(self, client):
        if client in self.client_name_map:
            return self.client_name_map[client]
        return None

    def get_available_clients(self):
        return self.client_map.keys()

    def add_client(self, client, address):
        client_name = self.generate_client_name(address)
        print 'Server: start connection with %s' % client_name
        self.inputs.append(client)
        self.client_map[client_name] = client
        self.client_name_map[client] = client_name

    def remove_client(self, client):
        comms.send(client, comms.EOT)
        client_name = self.get_client_name(client)
        print 'Server: end connection with %s' % client_name
        client.close()
        self.inputs.remove(client)
        del self.client_map[client_name]
        del self.client_name_map[client]

    def shutdown(self):
        self.server_lock.acquire()
        self.alive = False
        self.server.shutdown(socket.SHUT_RDWR)
        print 'Server: Shutting down...'
        for i in self.inputs:
            i.close()
        self.server_lock.release()
