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
        self.rate = rospy.Rate(5)

    ### signal connections to GUI ###
    def connect_update_gui(self, callback):
        self.update_data.connect(callback)

    ### callbacks ###
    def encoder_raw_callback(self, msg):
        self.data_struct.update_encoder_raw(msg.vector.x, msg.vector.y, msg.vector.z)
        
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

    def update_gui_data(self, data):
        if not self.lock.tryLock():
            print("WaterSampleRosThread: lock failed")
            return
        raw_pos_msg = self.ros_object.data_struct.encoder_raw
        self.lock.unlock()

        # display without decimal
        self.ui.rawX_DISP.display(int(raw_pos_msg.x))
        self.ui.rawY_DISP.display(int(raw_pos_msg.y))
        self.ui.rawZ_DISP.display(int(raw_pos_msg.z))

        

