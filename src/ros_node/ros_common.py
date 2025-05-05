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

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry

ROS_FREQ = 30  # Hz, enough for display and control


class TopicInfo:
    def __init__(self, topic, data_type):
        self.topic = topic
        self.data_type = data_type


# a list of topics used in this GUI:
GroundStationRosSubTopics = {
    "drone_imu": TopicInfo('mavros/imu/data', Imu),
    "drone_global_position": TopicInfo('mavros/global_position/global', NavSatFix),
    "drone_local_position": TopicInfo('state_estimator/local_position/odom/UAV0', Odometry),
    "esp_encoder": TopicInfo('/encoder/encoder_raw', TwistStamped)
    
}

GroundStationRosPubTopics = {
    "stepper_motor_mode_select":  TopicInfo('stepper/mode_select', Int32),
    "stepper_vel_direct": TopicInfo('stepper/vel_direct', Float32),
    "stepper_pos": TopicInfo('stepper/pos', Float32),
    "stepper_vel_auto": TopicInfo('stepper/vel_auto', Float32)
}

GroundStationRosServiceTopics = {
    "command_set_home": TopicInfo('mavros/set_mode', SetMode) # this may be bugged..
}


class EncoderInfo:
    def __init__(self):
        self.angleX = 0.0
        self.angleY = 0.0
        self.cable_len = 0.0
        self.angleX_vel = 0.0
        self.angleY_vel = 0.0
        self.cable_vel = 0.0


class StepperInfo:
    def __init__(self):
        self.mode = 0
        self.command_vel = 0

class IMUinfo:
    def __init__(self, x=0, y=0, z=0) -> None:
        self.roll = x
        self.pitch = y
        self.yaw = z


class GlobalPositionInfo:
    def __init__(self, latitude=0, longitude=0, altitude=0) -> None:
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude


class Vector3:
    def __init__(self, x=0, y=0, z=0) -> None:
        self.x = x
        self.y = y
        self.z = z


class BatteryInfo:
    def __init__(self, percentage=0, voltage=0) -> None:
        self.percentage = percentage
        self.voltage = voltage


class StateInfo:
    def __init__(self, connected=False, armed=False, manual_input=False, mode="", seconds=0) -> None:
        self.connected = connected
        self.armed = armed
        self.manual_input = manual_input
        self.mode = mode
        self.seconds = seconds
        self.total_seconds = 0


class AttitudeTarget:
    def __init__(self, roll=0, pitch=0, yaw=0, thrust=0) -> None:
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.thrust = thrust
