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

#!/usr/bin/env python

import sys
import subprocess
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer, QThread

import ROS_Node as ros_node
import GUI as gui

def start_rviz(config_file):
    try:
        subprocess.Popen(["rviz", "-d", config_file])
        print("RViz started:", config_file)
    except Exception as e:
        print("Failed to start RViz:", str(e))

if __name__ == "__main__":
    # init rosTrackingReference
    ros_node.rospy.init_node("GUI_Node_py")
    # start rviz
    # start_rviz("src/GUI/config.rviz")
    # define the window
    app = QtWidgets.QApplication(sys.argv)
    WaterSamplingGroundControlStation = QtWidgets.QTabWidget()
    ui = gui.Ui_WaterSamplingGroundControlStation()
    ui.setupUi(WaterSamplingGroundControlStation)
    # define ros threads
    rosSingleDroneThread = ros_node.SingleDroneRosThread(ui)
    rosSingleDroneThread.start()
    rosWaterSampleThread = ros_node.WaterSampleRosThread(ui)
    rosWaterSampleThread.start()
    
    # show the window
    WaterSamplingGroundControlStation.show()
    print("System Started")
    sys.exit(app.exec_())