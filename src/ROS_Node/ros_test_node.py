import rospy
from std_msgs.msg import String
from PyQt5.QtCore import QObject, pyqtSignal, QThread, QMutex
import Common
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped

class TestNode(QObject):
    ## define signals
    progress = pyqtSignal(int)
    pubMsg = pyqtSignal(int)
    updateData = pyqtSignal(int)
    
    def __init__(self):
        super().__init__()
        self.data = Common.CommonData()

        # define subscribers
        self.imu_sub = rospy.Subscriber('mavros/imu/data', Imu, callback=self.imu_sub)
        self.pos_global_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, callback=self.pos_global_sub)
        self.pos_local_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, callback=self.pos_local_sub)
        self.vel_sub = rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, callback=self.vel_sub)

        # define publishers
        self.pub = rospy.Publisher('/GroundStationTransmit', String, queue_size=10)
        self.pubMsg.connect(self.publish_position)

        # define services
        self.rate = rospy.Rate(10)
        
    ### define signal connections to / from gui ###
    def connectUpdateGUIData(self, callback):
        self.updateData.connect(callback)

    ### define callback functions from ros topics ###
    def imu_sub(self, msg): 
        # get orientation and convert to euler angles
        self.data.update_imu(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        
    def pos_global_sub(self, msg):
        self.data.update_global_pos(msg.latitude, msg.longitude, msg.altitude)
    
    def pos_local_sub(self, msg):
        self.data.update_local_pos(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def vel_sub(self, msg):
        self.data.update_vel(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)


    ### define gui input_signals ###
    def register_pub_task(self):
        self.pubMsg.emit(0)

    ### define publish functions to ros topics ###
    def publish_position(self):
        self.pub.publish('gui_test_msg')
    
    # main loop of ros node
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

    # define the signal-slot combination of ros and pyqt GUI
    def SetRosCallBack(self):
        # feedbacks from ros
        self.rosQtObject.connectUpdateGUIData(self.UpdateGUIData)

        # callbacks from GUI
        self.ui.SetHome.clicked.connect(self.rosQtObject.register_pub_task)

    # update GUI data
    def UpdateGUIData(self):
        if not self.lock.tryLock():
            print("Could not lock the mut")
            return
        imuMsg = self.rosQtObject.data.current_imu
        globalPosMsg = self.rosQtObject.data.current_global_pos
        localPosMsg = self.rosQtObject.data.current_local_pos
        velMsg = self.rosQtObject.data.current_vel
        self.lock.unlock()
        
        # accelerometer data
        self.ui.X_DISP.display("{:.2f}".format(imuMsg.roll, 2))
        self.ui.Y_DISP.display("{:.2f}".format(imuMsg.pitch, 2))
        self.ui.Z_DISP.display("{:.2f}".format(imuMsg.yaw, 2))

        # global & local position data
        self.ui.LatGPS_DISP.display("{:.2f}".format(globalPosMsg.latitude, 2))
        self.ui.LongGPS_DISP.display("{:.2f}".format(globalPosMsg.longitude, 2))
        self.ui.AltGPS_DISP.display("{:.2f}".format(globalPosMsg.altitude, 2))
        self.ui.RelX_DISP.display("{:.2f}".format(localPosMsg.x, 2))
        self.ui.RelY_DISP.display("{:.2f}".format(localPosMsg.y, 2))
        self.ui.AGL_DISP.display("{:.2f}".format(localPosMsg.z, 2))

        # velocity data
        self.ui.U_Vel_DISP.display("{:.2f}".format(velMsg.vx, 2))
        self.ui.V_Vel_DISP.display("{:.2f}".format(velMsg.vy, 2))
        self.ui.W_Vel_DISP.display("{:.2f}".format(velMsg.vz, 2))

    