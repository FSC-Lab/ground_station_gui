#!/usr/bin/env python
import ROS_Node.ros_common as ros_common
from PyQt5.QtCore import QMutex

class CommonData(): # store the data from the ROS nodes
    def __init__(self):
        # global msg_queue
        # msg_queue = Queue()
        self.msg = ""
        self.current_Time = ""
        self.current_battery_status = ""
        self.current_distance = ""

        self.current_imu = ros_common.IMUinfo()
        
        self.lock = QMutex()

    def update_imu(self, ax, ay, az):
        if not self.lock.tryLock():
            print("Could not lock the mutex")
            return
        self.current_imu.ax = ax
        self.current_imu.ay = ay
        self.current_imu.az = az
        self.lock.unlock()
        return

