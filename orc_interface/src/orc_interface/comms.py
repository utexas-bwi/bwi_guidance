#!/usr/bin/env python

import simplejson as json

START_TEXT = unichr(2)
END_TEXT = unichr(3)

MSG = START_TEXT
ACK = unichr(6)
NACK = unichr(15)
ENQ = unichr(5)
EOT = unichr(4)
ALL_NON_MSG_TYPES = [ACK, NACK, ENQ, EOT]

def convert(input):
    if isinstance(input, dict):
        return dict([(convert(key), convert(value)) 
                     for key, value in input.iteritems()])
    elif isinstance(input, list):
        return [convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

def send(channel, type, object=None):
    if type == MSG:
        object_str = json.dumps(object)
        frame_str = START_TEXT + object_str + END_TEXT
        channel.send(frame_str)
    else:
        if type in ALL_NON_MSG_TYPES: 
            channel.send(type)

def recv(channel):
    byte = channel.recv(1)
    try:
        byte = byte.encode('utf-8')
    except UnicodeDecodeError:
        return None, None
    if byte == START_TEXT:
        buf = ''
        while True:
            byte = channel.recv(1)
            try:
                byte = byte.encode('utf-8')
                if byte == END_TEXT:
                    break
            except UnicodeDecodeError:
                return None, None
            buf = buf + byte
        try:
            converted_data = convert(json.loads(buf))
        except ValueError:
            return MSG, None
        return MSG, converted_data
    else:
        if byte in ALL_NON_MSG_TYPES: 
            return byte, None
        else:
            return None, None
