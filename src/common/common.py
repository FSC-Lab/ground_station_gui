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

import ros_node.ros_common as ros_common
from PyQt5.QtCore import QMutex
from scipy.spatial.transform import Rotation


# store the data that are shared between the ros thread and the Qt main thread
class CommonData():
    def __init__(self):
        # basic drone information
        self.msg = ""
        self.current_Time = ""

        self.current_distance = ""
        self.total_distance = 0

        self.current_imu = ros_common.IMUinfo()
        self.current_global_pos = ros_common.GlobalPositionInfo()
        self.current_local_pos = ros_common.Vector3()
        self.current_vel = ros_common.Vector3()
        self.current_battery_status = ros_common.BatteryInfo()
        self.current_state = ros_common.StateInfo()
        self.current_attitude_target = ros_common.AttitudeTarget()
        self.indoor_mode = False

        # slung payload control
        # self.encoder_raw = ros_common.Vector3()
        # self.payload_pos = ros_common.Vector3()
        self.encoder_info = ros_common.EncoderInfo()

        self.lock = QMutex()  # this lock is to ensure that no data racing between the ros thread and the Qt main thread

    """
        all update functions should contain the lock feature to ensure thread safety

    """
    def update_imu(self, x, y, z, w):
        euler = self.quat_to_euler(x, y, z, w)

        if not self.lock.tryLock():
            return
        self.current_imu.roll = euler[0]
        self.current_imu.pitch = euler[1]
        self.current_imu.yaw = euler[2]
        self.lock.unlock()
        return

    def update_global_pos(self, latitude, longitude, altitude):
        if not self.lock.tryLock():
            return
        self.current_global_pos.latitude = latitude
        self.current_global_pos.longitude = longitude
        self.current_global_pos.altitude = altitude
        self.lock.unlock()
        return

    def update_local_pos(self, x, y, z):
        if not self.lock.tryLock():
            return
        self.current_local_pos.x = x
        self.current_local_pos.y = y
        self.current_local_pos.z = z
        self.lock.unlock()
        return

    def update_vel(self, vx, vy, vz):
        if not self.lock.tryLock():
            return
        self.current_vel.x = vx
        self.current_vel.y = vy
        self.current_vel.z = vz
        self.lock.unlock()
        return

    def update_bat(self, percentage, voltage):
        if not self.lock.tryLock():
            return
        self.current_battery_status.percentage = percentage
        self.current_battery_status.voltage = voltage
        self.lock.unlock()
        return

    def update_state(self, connected, armed, manual_input, mode, seconds):
        if not self.lock.tryLock():
            return
        self.current_state.connected = connected
        self.current_state.armed = armed
        self.current_state.manual_input = manual_input
        self.current_state.mode = mode
        self.current_state.seconds = seconds
        self.lock.unlock()
        return

    def update_attitude_target(self, x, y, z, w, thrust):
        euler = self.quat_to_euler(x, y, z, w)
        if not self.lock.tryLock():
            return
        self.current_attitude_target.roll = euler[0]
        self.current_attitude_target.pitch = euler[1]
        self.current_attitude_target.yaw = euler[2]
        self.current_attitude_target.thrust = thrust
        self.lock.unlock()
        return

    def quat_to_euler(self, x, y, z, w):
        nrm = abs(sum(it**2 for it in (x, y, z, w)) - 1.0)
        quat = [0, 0, 0, 1] if nrm > 1e-5 else [x, y, z, w]
        r = Rotation.from_quat(quat)
        euler = r.as_euler('xyz', degrees=True)

        # convert to 360 coordinates
        if euler[2] < 0:
            euler[2] = euler[2] + 360

        return euler

    def update_estimator_type(self, indoor_mode):
        if not self.lock.tryLock():
            return
        self.indoor_mode = indoor_mode
        self.lock.unlock()
        return

    # slung payload control

    def update_encoder(self, angleX, angleY, cable_len, angleX_vel, angleY_vel, cable_vel):
        if not self.lock.tryLock():
            return
        self.encoder_info.angleX = angleX
        self.encoder_info.angleY = angleY
        self.encoder_info.cable_len = cable_len
        self.encoder_info.angleX_vel = angleX_vel
        self.encoder_info.angleY_vel = angleY_vel
        self.encoder_info.cable_vel = cable_vel
        self.lock.unlock()
        return

    # def update_payload_pos(self, x, y, z):
    #     if not self.lock.tryLock():
    #         return
    #     self.payload_pos.x = x
    #     self.payload_pos.y = y
    #     self.payload_pos.z = z
    #     self.lock.unlock()
    #     return
