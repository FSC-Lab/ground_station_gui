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
