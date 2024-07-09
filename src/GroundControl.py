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
    start_rviz("src/GUI/config.rviz")
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