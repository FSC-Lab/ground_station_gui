import rospy
from std_msgs.msg import String
from PyQt5.QtCore import QObject, pyqtSignal, QThread, QMutex
import Common
from sensor_msgs.msg import Imu, NavSatFix

from queue import Queue
import ROS_Node as ros_node

class TestNode(QObject):
    ## define signals
    progress = pyqtSignal(int)
    pubMsg = pyqtSignal(int)
    updateData = pyqtSignal(int)
    
    def __init__(self):
        super().__init__()
        self.data = Common.CommonData()

        # define subscribers
        self.chat_sub = rospy.Subscriber('chatter', String, callback=self.chatter_sub)
        self.imu_sub = rospy.Subscriber('mavros/imu/data', Imu, callback=self.imu_sub)
    
        # define publishers
        self.pub = rospy.Publisher('/GroundStationTransmit', String, queue_size=10)
        self.pubMsg.connect(self.publish_position)

        # define services
        self.rate = rospy.Rate(5)
        
    ## define signal connections to gui (slots)

    def connectChatCallBack(self, callback):
        self.progress.connect(callback)

    def connectUpdateGUIData(self, callback):
        self.updateData.connect(callback)

    ## define callback functions from ros topics

    def chatter_sub(self, msg):
        # emit a signal to the gui to update plots
        Common.msg_queue.put(msg)
        self.progress.emit(0)

    def imu_sub(self, msg):
        ## update imu reading
        self.data.update_imu(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)     

    def publish_position(self):
        self.pub.publish('gui_test_msg')

    def register_pub_task(self):
        print("registering pub task")
        self.pubMsg.emit(0)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.updateData.emit(0)
                self.rate.sleep()   ## sleep for 0.2 seconds
            except rospy.ROSInterruptException:
                print("ROS Shutdown Requested")
                break

class RosThread:
    def __init__(self, ui):
        super().__init__()
        self.rosQtObject = TestNode()
        self.thread = QThread()

        # setup signals
        self.ui = ui
        self.SetRosCallBack()

        # move and start thread
        self.rosQtObject.moveToThread(self.thread)
        
        self.lock = self.rosQtObject.data.lock
        self.thread.started.connect(self.rosQtObject.run)

    def start(self):
        self.thread.start()

    def stop(self):
        self.thread.quit()
        self.thread.wait()

    ## define callback functions
    def UpdateGUIData(self):
        if not self.lock.tryLock():
            print("Could not lock the mutex")
            return
        imuMsg = self.rosQtObject.data.current_imu
        self.lock.unlock()
        self.ui.X_DISP.display("{:.2f}".format(imuMsg.ax, 2))

    ## define the signal-slot combination of ros and pyqt GUI
    def SetRosCallBack(self):
        # feedbacks from ros
        self.rosQtObject.connectUpdateGUIData(self.UpdateGUIData)

        # callbacks from GUI
        self.ui.SetHome.clicked.connect(self.rosQtObject.register_pub_task)