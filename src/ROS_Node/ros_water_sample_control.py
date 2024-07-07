import rospy
from PyQt5.QtCore import QObject, pyqtSignal, QThread
from PyQt5.QtWidgets import QMessageBox
import Common

from geometry_msgs.msg import Vector3Stamped

class WaterSampleRosNode(QObject):
    update_data = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.data_struct = Common.CommonData()
        # define subscribers
        self.encoder_raw_sub = rospy.Subscriber("/encoder/position_raw", Vector3Stamped, self.encoder_raw_callback)
        #self.encoder_raw_vel_sub = rospy.Subscriber("/encoder/speed_raw", Vector3Stamped, self.encoder_raw_spd_callback)
        self.payload_pos_sub = rospy.Subscriber("/encoder/position_payload", Vector3Stamped, self.payload_pos_callback)
        #self.payload_pos_vel_sub = rospy.Subscriber("/encoder/speed_payload", Vector3Stamped, self.payload_pos_spd_callback)

        # define publishers
        self.cmd_length_pub = rospy.Publisher("/encoder/setpoint_length", Vector3Stamped, queue_size=10)


        self.rate = rospy.Rate(5)

    ### signal connections to GUI ###
    def connect_update_gui(self, callback):
        self.update_data.connect(callback)

    ### callbacks ###
    def encoder_raw_callback(self, msg):
        self.data_struct.update_encoder_raw(msg.vector.x, msg.vector.y, msg.vector.z)

    def payload_pos_callback(self, msg):
        self.data_struct.update_payload_pos(msg.vector.x, msg.vector.y, msg.vector.z)

    def pub_payload_length(self, len):
        length_msg = Vector3Stamped()
        length_msg.header.stamp = rospy.Time.now()
        length_msg.vector.z = len
        self.cmd_length_pub.publish(length_msg)

        
    def run(self):
        while not rospy.is_shutdown():
            try:
                self.update_data.emit(0)
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

class WaterSampleRosThread():
    def __init__(self, ui):
        super().__init__()
        self.ros_object = WaterSampleRosNode()
        self.thread =  QThread()

        # setup
        self.ui = ui
        self.set_ros_callbacks()

        # start ros thread
        self.ros_object.moveToThread(self.thread)
        self.lock = self.ros_object.data_struct.lock
        self.thread.started.connect(self.ros_object.run)

    def start(self):
        self.thread.start()

    def set_ros_callbacks(self):
        # ros2gui
        self.ros_object.connect_update_gui(self.update_gui_data)

        # gui2ros
        self.ui.btnSendLength.clicked.connect(self.send_length_request)
        self.ui.btnUp.clicked.connect(self.send_up_incr_request)
        self.ui.btnDown.clicked.connect(self.send_down_incr_request)

    def update_gui_data(self, data):
        if not self.lock.tryLock():
            print("WaterSampleRosThread: lock failed")
            return
        self.raw_pos_msg = self.ros_object.data_struct.encoder_raw
        self.payload_pos_msg = self.ros_object.data_struct.payload_pos
        self.lock.unlock()

        # display without decimal
        self.ui.rawX_DISP.display(int(self.raw_pos_msg.x))
        self.ui.rawY_DISP.display(int(self.raw_pos_msg.y))
        self.ui.rawZ_DISP.display(int(self.raw_pos_msg.z))

        self.ui.PayloadX_DISP.display(int(self.payload_pos_msg.x))
        self.ui.PayloadY_DISP.display(int(self.payload_pos_msg.y))
        self.ui.PayloadZ_DISP.display(int(self.payload_pos_msg.z))

    def display_current_length(self):
        self.ui.length_CMD.setText(str(self.payload_pos_msg.z))

    def send_length_request(self):
        self.ros_object.pub_payload_length(float(self.ui.length_CMD.text()))

    def send_down_incr_request(self):
        self.send_stop_request()
        abs_length = self.payload_pos_msg.z + float(self.ui.increment_CMD.text())
        self.ros_object.pub_payload_length(abs_length)

    def send_up_incr_request(self):
        self.send_stop_request()
        abs_length = self.payload_pos_msg.z - float(self.ui.increment_CMD.text())
        self.ros_object.pub_payload_length(abs_length)

    def send_stop_request(self):
        self.ros_object.pub_payload_length(float(self.ui.length_CMD.text()))

