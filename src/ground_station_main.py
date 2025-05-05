#!/usr/bin/env python
'''
MIT License

Copyright (c) 2024 FSC Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

import sys
import subprocess
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer, QThread

import ros_node as ros_node
import gui as gui


if __name__ == "__main__":
    # init rosTrackingReference
    ros_node.rospy.init_node("GUI_Node_py")
    # define the window
    app = QtWidgets.QApplication(sys.argv)
    ground_station_gui = QtWidgets.QTabWidget()
    ui = gui.Ui_ground_station()
    ui.setupUi(ground_station_gui)
    # define ros threads
    rosSingleDroneThread = ros_node.SingleDroneRosThread(ui)
    rosSingleDroneThread.start()
    # rosWaterSampleThread = ros_node.WaterSampleRosThread(ui)
    # rosWaterSampleThread.start()

    # show the window
    ground_station_gui.show()
    print("ground station gui started...")
    sys.exit(app.exec_())