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

This is the ROS 1 node class definiting all publishers,
subscribers and Qt singals.
"""

import rospy
from PyQt5.QtCore import QObject, pyqtSignal, QThread
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QStringListModel
from PyQt5 import QtWidgets
import ros_node
import threading
import common


# augment the ros subscriber with watchdog feature
class SubscriberWatchdog:
    def __init__(self, topic_name, msg_type, callback_func, timeout_sec):
        self.timeout = timeout_sec
        self.last_msg_time = rospy.Time.now()
        self.lock = threading.Lock()
        self.callback_func = callback_func
        self.sub = rospy.Subscriber(topic_name, msg_type, self.callback)

    def callback(self, msg):
        with self.lock:
            self.last_msg_time = rospy.Time.now()
            self.callback_func(msg)

    def check(self):
        with self.lock:
            time_since_last = (rospy.Time.now() - self.last_msg_time).to_sec()
        return time_since_last < self.timeout

    def stop(self):
        self.sub.unregister()


class GroundStationRos1Node(QObject):
    # signals used in Qt to do synchronization

    update_data = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.common_data = common.CommonData()
        # subscribers
        self.esp_encoder_sub = SubscriberWatchdog(ros_node.GroundStationRosTopics["esp_encoder"].topic,
                                                  ros_node.GroundStationRosTopics["esp_encoder"].data_type,
                                                  callback=self.esp_msg_sub)

        # only use watchdog when subscribing the drone state is enough

        # publishers
        self.stepper_motor_mod_pub = rospy.Publisher(ros_node.GroundStationRosTopics["stepper_motor_mode_select"].topic,
                                                     ros_node.GroundStationRosTopics["stepper_motor_mode_select"].data_type,
                                                     queue_size=10)
    
        self.stepper_vel_direct_pub = rospy.Publisher(ros_node.GroundStationRosTopics["stepper_vel_direct"].topic,
                                                      ros_node.GroundStationRosTopics["stepper_vel_direct"].data_type,
                                                      queue_size=10)

        self.stepper_pos_pub = rospy.Publisher(ros_node.GroundStationRosTopics["stepper_pos"].topic,
                                               ros_node.GroundStationRosTopics["stepper_pos"].data_type,
                                               queue_size=10)

        self.stepper_pos_pub = rospy.Publisher(ros_node.GroundStationRosTopics["stepper_vel_auto"].topic,
                                               ros_node.GroundStationRosTopics["stepper_vel_auto"].data_type,
                                               queue_size=10)

        # ros setup
        self.rate = rospy.Rate(ros_node.ROS_FREQ)
        
        # local variable setup
        self.encoder_info = ros_node.EncoderInfo()

        # self.res = TwistStamped()
        # self.is_publishing = False
        self.lock = threading.Lock()
    
    # ### define signal connections to / from gui ###100
    # def connect_update_gui(self, callback):
    #     self.update_data.connect(callback)

    ### define callback functions from ros topics ###
    def esp_msg_sub(self, msg):
        self.common_data.update_encoder(msg.twist.linear.x,
                                        msg.twist.linear.y,
                                        msg.twist.linear.z,
                                        msg.twist.angular.x,
                                        msg.twist.angular.y,
                                        msg.twist.angular.z)

    def publish_motor_mode(self, mode):
        pass
        # for i in range(4):
        #     self.motor_msg.motors[i] = throttle
        # self.throttle_pub.publish(self.motor_msg)

    def get_encoder_status(self):
        return self.esp_encoder_sub.check()

    # main loop of ros node
    def run(self):
        while not rospy.is_shutdown():
            try:
                self.update_data.emit(0) # notify Qt in every iteration
                self.rate.sleep()
            except rospy.ROSInterruptException:
                print("ROS Shutdown Requested")
                break
        self.esp_encoder_sub.stop()
