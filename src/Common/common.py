#!/usr/bin/env python
import ROS_Node.ros_common as ros_common
from PyQt5.QtCore import QMutex
from scipy.spatial.transform import Rotation


class CommonData(): # store the data from the ROS nodes
    def __init__(self):
        self.msg = ""
        self.current_Time = ""
        self.current_battery_status = ""
        self.current_distance = ""

        self.current_imu = ros_common.IMUinfo()
        self.current_global_pos = ros_common.GlobalPositionInfo()
        self.current_local_pos = ros_common.LocalPositionInfo()
        self.current_vel = ros_common.VelocityInfo()

        self.lock = QMutex()

    def update_imu(self, x, y, z, w):
        ## convert quaternion to euler angles
        r = Rotation.from_quat([x, y, z, w])
        euler = r.as_euler('xyz', degrees=True)
        # convert to 360 coordinates

        euler[2] = (90 - euler[2]) % 360
    
        if not self.lock.tryLock():
            print("Could not lock the mutex")
            return
        self.current_imu.roll = euler[0]
        self.current_imu.pitch = euler[1]
        self.current_imu.yaw = euler[2]
        self.lock.unlock()
        return
  

    def update_global_pos(self, latitude, longitude, altitude):
        if not self.lock.tryLock():
            print("Could not lock the mutex")
            return
        self.current_global_pos.latitude = latitude
        self.current_global_pos.longitude = longitude
        self.current_global_pos.altitude = altitude
        self.lock.unlock()
        return

    def update_local_pos(self, x, y, z):
        if not self.lock.tryLock():
            print("Could not lock the mutex")
            return
        self.current_local_pos.x = x
        self.current_local_pos.y = y
        self.current_local_pos.z = z
        self.lock.unlock()
        return
    
    def update_vel(self, vx, vy, vz):
        if not self.lock.tryLock():
            print("Could not lock the mutex")
            return
        self.current_vel.vx = vx
        self.current_vel.vy = vy
        self.current_vel.vz = vz
        self.lock.unlock()
        return
    
    


