import rospy
from PyQt5.QtCore import QObject, pyqtSignal, QThread
from PyQt5.QtWidgets import QMessageBox
import Common

from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32MultiArray

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
        self.cmd_stepper_pub = rospy.Publisher("/stepper/setpoint", Float32MultiArray, queue_size=1)
        self.cable_test_pub = rospy.Publisher("/stepper/test", Float32MultiArray, queue_size=1)

        self.rate = rospy.Rate(5)

    ### signal connections to GUI ###
    def connect_update_gui(self, callback):
        self.update_data.connect(callback)

    ### callbacks ###
    def encoder_raw_callback(self, msg):
        self.data_struct.update_encoder_raw(msg.vector.x, msg.vector.y, msg.vector.z)

    def payload_pos_callback(self, msg):
        self.data_struct.update_payload_pos(msg.vector.x, msg.vector.y, msg.vector.z)

    def pub_stepper_cmd(self, len, spd):
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [len, spd]
        self.cmd_stepper_pub.publish(cmd_msg)

    def pub_stepper_test(self, amplitude, speed, startCMD):
        test_msg = Float32MultiArray()
        if startCMD:
            test_msg.data = [1, amplitude, speed]
            self.cable_test_pub.publish(test_msg)
            return
        test_msg.data = [0, amplitude, speed]
        
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
        self.ui.btnSendCMD.clicked.connect(self.send_cable_cmd_req)
        self.ui.btnUp.clicked.connect(self.send_up_incr_request)
        self.ui.btnDown.clicked.connect(self.send_down_incr_request)
        self.ui.btnStartTest.clicked.connect(lambda: self.send_test_request(True))
        self.ui.btnStopTest.clicked.connect(lambda: self.send_test_request(False))


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

        self.ui.PayloadX_DISP.display(float(self.payload_pos_msg.x))
        self.ui.PayloadY_DISP.display(float(self.payload_pos_msg.y))
        self.ui.PayloadZ_DISP.display(float(self.payload_pos_msg.z))

    def send_cable_cmd_req(self):
        self.ros_object.pub_stepper_cmd(float(self.ui.length_CMD.text()), float(self.ui.speed_CMD.text()))

    def send_down_incr_request(self):
        self.send_stop_request()
        abs_length = self.payload_pos_msg.z + float(self.ui.increment_CMD.text())
        self.ros_object.pub_stepper_cmd(abs_length, 0.5) # 0.5 m/s

    def send_up_incr_request(self):
        self.send_stop_request()
        abs_length = self.payload_pos_msg.z - float(self.ui.increment_CMD.text())
        self.ros_object.pub_stepper_cmd(abs_length, 0.5)

    def send_stop_request(self):
        self.ros_object.pub_stepper_cmd(float(self.ui.length_CMD.text()), 0)

    ## testing
    def send_test_request(self, startCMD=True):
        self.send_stop_request()
        self.ros_object.pub_stepper_cmd(float(self.ui.test_amp_CMD.text()), float(self.ui.test_speed_CMD.text()), startCMD)