#!/usr/bin/env python
from queue import Queue

msg_queue = Queue()
imu_queue = Queue()


def init():
    global msg, current_Time, current_imu, current_battery_status, current_distance
    # global msg_queue
    # msg_queue = Queue()
    msg = ""
    current_Time = ""
    current_imu = ""
    current_battery_status = ""
    current_distance = ""

