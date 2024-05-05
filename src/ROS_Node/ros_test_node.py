import rospy
from PyQt5.QtCore import QObject, pyqtSignal, QThread
import Common
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandHome, CommandHomeRequest, CommandLong
from mavros_msgs.msg import State


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
        self.bat_sub = rospy.Subscriber('mavros/battery', BatteryState, callback=self.bat_sub)
        self.status_sub = rospy.Subscriber('mavros/state', State, callback=self.status_sub)

        # define publishers / services
        self.set_home_service = rospy.ServiceProxy('mavros/cmd/set_home', CommandHome)
        self.arming_service = rospy.ServiceProxy('mavros/cmd/command', CommandLong)

        # other
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

    def bat_sub(self, msg):
        self.data.current_battery_status = msg.percentage

    def status_sub(self, msg):
        self.data.update_state(msg.connected, msg.armed, msg.manual_input, msg.mode)

    ### define gui input_signals ###
    def send_set_home_request(self):
        home_position = CommandHomeRequest()
        home_position.latitude = self.data.current_global_pos.latitude
        home_position.longitude = self.data.current_global_pos.longitude
        home_position.altitude = self.data.current_global_pos.altitude
        response = self.set_home_service(home_position)
        print(response)

    ### define publish / service functions to ros topics ###
    def send_arming_request(self, arm, param2):
        response = self.arming_service(command=400, confirmation=0, param1 = arm, param2 = param2)
        print(response)
    
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
        self.ui.SetHome.clicked.connect(self.rosQtObject.send_set_home_request)
        self.ui.SimulationMode.stateChanged.connect(self.toggle_simulation_mode)
        self.ui.ARM.clicked.connect(lambda: self.rosQtObject.send_arming_request(True, 0))
        self.ui.DISARM.clicked.connect(lambda: self.rosQtObject.send_arming_request(False, 0))
        self.ui.EmergencyStop.clicked.connect(lambda: self.rosQtObject.send_arming_request(False, 21196))

    # update GUI data
    def UpdateGUIData(self):
        if not self.lock.tryLock():
            print("Could not lock the mutex")
            return
        imuMsg = self.rosQtObject.data.current_imu
        globalPosMsg = self.rosQtObject.data.current_global_pos
        localPosMsg = self.rosQtObject.data.current_local_pos
        totDist = self.rosQtObject.data.total_distance

        velMsg = self.rosQtObject.data.current_vel

        batMsg = self.rosQtObject.data.current_battery_status
        stateMsg = self.rosQtObject.data.current_state
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
        self.ui.Meters_DISP.display("{:.2f}".format(totDist, 2))

        # velocity data
        self.ui.U_Vel_DISP.display("{:.2f}".format(velMsg.vx, 2))
        self.ui.V_Vel_DISP.display("{:.2f}".format(velMsg.vy, 2))
        self.ui.W_Vel_DISP.display("{:.2f}".format(velMsg.vz, 2))

        # state updates
        self.ui.StateARM.setText("Armed" if stateMsg.armed else "Disarmed")
        self.ui.StateARM.setStyleSheet("color: red" if stateMsg.armed else "color: green")
        self.ui.StateConnected.setText("Connected" if stateMsg.connected else "Disconnected")
        self.ui.StateConnected.setStyleSheet("color: green" if stateMsg.connected else "color: red")
        self.ui.StateMode.setText(stateMsg.mode) 

        # misc data
        if batMsg: # takes long to initialize
            if self.ui.BatInd.isTextVisible() == False:
                self.ui.BatInd.setTextVisible(True)
            self.ui.BatInd.setValue(int(batMsg)*100)

    ### callback functions for modifying GUI elements ###
    def toggle_simulation_mode(self, state):
        if state == 2:
            print("Arming controls available")
            self.ui.ARM.setEnabled(True)
            self.ui.DISARM.setEnabled(True)
        else:
            print("Arming controls disabled")
            self.ui.ARM.setEnabled(False)
            self.ui.DISARM.setEnabled(False)
        
    