import rospy
from std_msgs.msg import String
from PyQt5.QtCore import QObject, pyqtSignal, QThread
import Common
from sensor_msgs.msg import Imu
from queue import Queue
import ROS_Node as ros_node

class TestNode(QObject):
    progress = pyqtSignal(int)
    imuUpdate = pyqtSignal(int)
    pubMsg = pyqtSignal(int)
    def __init__(self):
        super().__init__()
        # define subscribers
        self.chat_sub = rospy.Subscriber('chatter', String, callback=self.chatter_sub)
        self.imu_sub = rospy.Subscriber('mavros/imu/data', Imu, callback=self.imu_sub)
        # define publishers
        self.pub = rospy.Publisher('/GroundStationTransmit', String, queue_size=10)
        self.pubMsg.connect(self.publish_position)
        # define services...
        self.rate = rospy.Rate(5)

    def connectChatCallBack(self, callback):
        self.progress.connect(callback)

    def connectUpdateImuReading(self, callback):
        self.imuUpdate.connect(callback)

    def chatter_sub(self, msg):
        # emit a signal to the gui to update plots
        Common.msg_queue.put(msg)
        self.progress.emit(0)

    def imu_sub(self, msg):
        Common.imu_queue.put(ros_node.IMUinfo(
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ))
        self.imuUpdate.emit(0)

    def publish_position(self):
        self.pub.publish('gui_test_msg')

    def register_pub_task(self):
        self.pubMsg.emit(0)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


class RosThread:
    def __init__(self, ui):
        self.thread = QThread()
        self.rosQtObject = TestNode()
        self.rosQtObject.moveToThread(self.thread)
        self.ui = ui
        self.SetRosCallBack()

    def start(self):
        self.thread.start()

    def UpdateImuReading(self):
        imuMsg = Common.imu_queue.get()
        self.ui.X_DISP.display("{:.2f}".format(imuMsg.ax, 2))

    # define the signal-slot combination of ros and pyqt GUI
    def SetRosCallBack(self):
        # feebacks
        self.rosQtObject.connectUpdateImuReading(self.UpdateImuReading)
        # buttom callbacks
        self.ui.SetHome.clicked.connect(self.rosQtObject.register_pub_task)