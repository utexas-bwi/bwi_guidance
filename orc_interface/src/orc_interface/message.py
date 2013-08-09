#!/usr/bin/env python

INVOKE = 'invoke'
RETURN = 'return'
CANCEL = 'cancel'
FAILURE = 'failure'

class InterfaceMessage:

    def __init__(self, client_name=None, data=None):
        if client_name != None:
            self.request = False
            self.valid = False
            self.client_name = client_name
            self.representation = str(data)
            self.type = data['msgType']
            try:
                if data['msgType'] == INVOKE:
                    self.request = True
                    self.call_id = data['callId'] #id
                    self.target = data['target'] #str
                    self.method = data['method'] #str
                    self.args = data['args'] #list
                    self.valid = True
                elif data['msgType'] == CANCEL:
                    self.request = True
                    self.call_id = data['callId'] #id
                    self.valid = True
                elif data['msgType'] == RETURN:
                    self.call_id = data['callId'] #id
                    self.value = data['value'] #int
                    self.valid = True
                elif data['msgType'] == FAILURE:
                    self.valid = True
                    self.call_id = data['callId'] #id
                    self.fail_type = data['failType'] #int
                    self.message = data['message'] #str
                    self.causes = data['causes'] #list ??
                    self.valid = True
            except KeyError:
                pass
            except TypeError:
                pass

    def __str__(self):
        return 'Client: ' + self.client_name + ', Data: ' + self.representation

    def is_valid(self):
        return self.valid

    def is_request(self):
        return self.request

    def to_dict(self):
        ret_data = dict()
        ret_data['msgType'] = self.type
        ret_data['callId'] = self.call_id
        if self.type == INVOKE:
            ret_data['target'] = self.target
            ret_data['method'] = self.method
            ret_data['args'] = self.args
        elif self.type == RETURN:
            ret_data['value'] = self.value
        elif self.type == FAILURE:
            ret_data['failType'] = self.fail_type
            ret_data['message'] = self.message
            ret_data['causes'] = self.causes
        return ret_data

       


