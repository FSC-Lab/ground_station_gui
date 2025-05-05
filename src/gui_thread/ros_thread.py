#!/usr/bin/env python
"""
MIT License

Copyright (c) 2025 Flight Systems and Control Laboratory

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

This file defines the thread to handle the interactions between ros and Qt gui

"""
from PyQt5.QtCore import QObject, pyqtSignal, QThread
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QStringListModel
from PyQt5 import QtWidgets
import gui_thread
import ros_node


class GuiRosThread:
    def __init__(self, ui, ros_version=None):
        super().__init__()
        self.setup_ros_node(ros_version)
        self.thread = QThread()

        # setup signals
        self.ui = ui
        self.set_ros_callbacks()  # setup the interaction between ros and Qt gui

        self.setup_and_start_thread(self.ros_object)

        # determine the current script location
        # self.csv_directory = os.path.join(os.path.dirname(__file__), "../../throttle_profile")

        # canvas
        # Add Matplotlib canvas to the UI
        # self.canvas = gui.MplCanvas(parent=ui, width=7, height=3, dpi=50)
        # self.layout = QtWidgets.QVBoxLayout(ui.plotWidget)  # Replace plotWidget with your placeholder widget's name
        # self.layout.addWidget(self.canvas)
        
        # TO DO: read geofence

    def setup_ros_node(self):
        if self.ros_version is None:
            self.ros_version = gui_thread.detect_ros_version()  # if version is not set, use auto detection.
        print(f"current ros version is {self.ros_version}")
        if self.ros_version == 'ROS 1':
            self.ros_object = ros_node.GroundStationRos1Node()
            return
        if self.ros_version == 'ROS 2':
            # right now ROS 2 node is not completed
            # so for now it will raise an error
            raise Exception("ROS 2 detected! However, ROS2 version is not completed")
            # return
        raise Exception("Error determing the ROS version on this computer!\
                        You can try manually setting the ROS version.")

    def setup_and_start_thread(self, ros_object):
        # move and start thread
        ros_object.moveToThread(self.thread)
        self.lock = ros_object.common_data.lock
        self.thread.started.connect(ros_object.run)

    def start(self):
        self.thread.start()
        # define the signal-slot combination of ros and pyqt GUI

    def set_ros_callbacks(self):
        # feedbacks from ros
        # self.ros_object.connect_update_gui(self.update_gui_data)

        # # throttle slider config
        # self.ui.Throttle.valueChanged.connect(self.update_throttle_value)
        # self.ui.Throttle.setEnabled(False)

        # # buttom config
        # self.ui.arm_buttom.clicked.connect(self.on_arm_pressed)
        # self.ui.enable_manual.stateChanged.connect(self.on_check_manual_control)
        # self.ui.profile_start.clicked.connect(self.throttle_profile_object.activate)
        # self.ui.profile_stop.clicked.connect(self.throttle_profile_object.reset)

        # # throttle profile load
        # self.ui.profile_scan.clicked.connect(self.scan_csv_files)
        
        # self.model = QStringListModel()  # To store and display file names
        # self.ui.csv_view.setModel(self.model)
        
        # self.ui.profile_load.clicked.connect(self.load_csv_file)
        pass

    # get the throttle command from the slider if in manual mode
    def update_throttle_value(self):
        pass
        # if self.enable_maunal_thottle:
        #     self.thottle = self.ui.Throttle.value()
        #     thrustData.update_throttle(self.thottle)
    
    # by pressing the arm buttom, arm the motor
    def on_arm_pressed(self):
        pass
        # self.thottle = 0
        # if self.armed:
        #     self.armed = False
        #     self.ui.arm_buttom.setText("Arm")
        #     self.ros_object.publish_arm(False)
        #     # set the status text to armed
        # else:
        #     self.ui.arm_buttom.setText("Disarm")
        #     self.armed = True
        #     self.ros_object.publish_arm(True)
        #     # set the status text to disarmed

    # update GUI data
    def update_gui_data(self):
        pass
        # self.throttle_slider_control(self.armed, self.enable_maunal_thottle)
        # self.update_status_labels()
        # # update throttle from throttle profile
        # self.update_throttle_value_profile()
        # self.update_profile_control_button()
        # self.update_profile_status()
        # # send throttle cmd if certain conditions are met
        # self.send_throttle_cmd()
        # if len(self.ros_object.esc_rpm.rpm) >= 2:
        #     self.ui.esc_rpm_display.display("{:.0f}".format(self.ros_object.esc_rpm.rpm[1], 2))
        
        # if not self.lock.tryLock():
        #     print("MotorStandRosThread: lock failed")
        #     return
        #     # publish result
        #     # self.res.twist.linear.x = data[0] # throttle command
        #     # self.res.twist.linear.y = data[1] # torque
        #     # self.res.twist.linear.z = data[2] # thrust
        #     # self.res.twist.angular.x = data[3] # esc rpm
        #     # self.res.twist.angular.y = data[4] # optical rpm
        #     # self.res.twist.angular.z = data[5] # voltage
        
        # esc_rpm = 0
        # if len(self.ros_object.esc_rpm.rpm) >= 2:
        #     esc_rpm = self.ros_object.esc_rpm.rpm[1]
        # data_ = [self.thottle, thrustData.torque, thrustData.thrust,
        #         esc_rpm, thrustData.motorOpticalSpeed, thrustData.voltage]
        # self.lock.unlock()
        # self.ros_object.publish_res(data_)