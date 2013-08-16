#!/usr/bin/env python

import orc_interface
from PyQt4 import QtGui

class ListInterfaceWidget(QtGui.QWidget):

    def __init__(self, robots, available_robots):
        QtGui.QWidget.__init__(self)
        self.robots = robots
        self.available_robots = available_robots
        names = ['Disable ' + robot for robot in self.robots]
        pos = [(i, 0) for i in range(len(names))]

        grid = QtGui.QGridLayout()
        count = 0
        for name in names:
            button = QtGui.QPushButton(name)
            button.clicked.connect(self.button_clicked)
            grid.addWidget(button, pos[count][0], pos[count][1])
            count = count + 1

        self.setLayout(grid)
        self.setWindowTitle('Robots')
        self.show()

    def button_clicked(self):
        sender = self.sender()
        sender_text = str(sender.text())
        text_list = sender_text.split(' ')
        if text_list[0] == 'Disable':
            self.available_robots.remove(text_list[1])
            sender.setText('Enable ' + text_list[1])
        else:
            if text_list[1] not in self.available_robots:
                self.available_robots.append(text_list[1])
                sender.setText('Disable ' + text_list[1])

class ListInterface:

    def __init__(self, robots):

        #TODO select only relevant targets here?
        self.robots = robots
        self.available_robots = list(self.robots)
        self.display = ListInterfaceWidget(self.robots, self.available_robots)

    def invoke(self, call_id, target, args):
        pass
        
    def cancel(self, call_id):
        pass

    def check_status(self, call_id):
        ret_message = orc_interface.InterfaceMessage()
        ret_message.type = orc_interface.RETURN
        ret_message.call_id = call_id
        ret_message.value = self.available_robots
        return True, ret_message
