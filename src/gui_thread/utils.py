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

"""

import os
import json


# a function to detect the version of ros
def detect_ros_version():
    ros_version = os.environ.get('ROS_VERSION')
    if ros_version == '1':
        return 'ROS 1'
    elif ros_version == '2':
        return 'ROS 2'
    else:
        # Fallback: try importing modules
        try:
            import rclpy
            return 'ROS 2 (detected via import)'
        except ImportError:
            pass

        try:
            import rospy
            return 'ROS 1 (detected via import)'
        except ImportError:
            pass

        return 'Unknown ROS version (neither environment variable nor imports worked)'


# read geofence from json file
def load_geo_fence(filename):
    config = [0, 0, 0]
    with open('src/ROS_Node/geofence.json') as f:
        geofence = json.load(f)
        config[0] = geofence['x']
        config[1] = geofence['y']
        config[2] = geofence['z']
    return config
