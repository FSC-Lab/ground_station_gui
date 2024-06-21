import rospy
from PyQt5.QtCore import QObject, pyqtSignal, QThread
from PyQt5.QtWidgets import QMessageBox
import Common
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from mavros_msgs.srv import CommandHome, CommandHomeRequest, CommandLong, SetMode
from mavros_msgs.msg import State, AttitudeTarget
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import json
from tracking_control.msg import TrackingReference
from std_msgs.msg import Bool

class SingleDroneRosNode(QObject):
    ## define signals
    updateData = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.data = Common.CommonData()
        # define subscribers
        self.imu_sub = rospy.Subscriber('mavros/imu/data', Imu, callback=self.imu_sub)
        self.pos_global_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, callback=self.pos_global_sub)
        self.pos_local_adjusted_sub = rospy.Subscriber('mavros/local_position/adjusted', PoseStamped, callback=self.pos_local_sub)
        self.vel_sub = rospy.Subscriber('mavros/local_position/odom/UAV0', Odometry, callback=self.vel_sub)
        self.bat_sub = rospy.Subscriber('mavros/battery', BatteryState, callback=self.bat_sub)
        self.status_sub = rospy.Subscriber('mavros/state', State, callback=self.status_sub)
        self.commanded_attitude_sub = rospy.Subscriber('mavros/setpoint_raw/attitude', AttitudeTarget, callback=self.commanded_attitude_sub)
        self.estimator_type_sub = rospy.Subscriber('/estimator_type', Bool, callback=self.estimator_type_sub)

    

        # define publishers / services
        self.coords_pub = rospy.Publisher('tracking_controller/target', TrackingReference, queue_size=10)

        self.set_home_override_service = rospy.ServiceProxy('mavros/override_set_home', Empty)
        self.set_home_service = rospy.ServiceProxy('mavros/cmd/set_home', CommandHome)

        self.arming_service = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
        self.land_service = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
        self.set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # other
        self.rate = rospy.Rate(5)

        # read geofence from json file
        with open('src/ROS_Node/geofence.json') as f:
            geofence = json.load(f)
            self.config = geofence
            print("Geofence loaded: ", self.config)
        
    ### define signal connections to / from gui ###
    def connectUpdateGUIData(self, callback):
        self.updateData.connect(callback)

    def connectGUIPub(self, callback):
        self.pubGUIsig.connect(callback)

    def register_gui_pub(self):
        self.pubGUIsig.emit(0)

    def publish_coordinates(self, x, y, z, yaw):
        point = TrackingReference()
        point.header.stamp = rospy.Time.now()
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = z
        point.yaw = yaw
        self.coords_pub.publish(point)

    ### define callback functions from ros topics ###
    def imu_sub(self, msg): 
        # get orientation and convert to euler angles
        self.data.update_imu(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w) 
        
    def pos_global_sub(self, msg):
        self.data.update_global_pos(msg.latitude, msg.longitude, msg.altitude)
    
    def pos_local_sub(self, msg):
        self.data.update_local_pos(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def vel_sub(self, msg):
        self.data.update_vel(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)

    def bat_sub(self, msg):
        self.data.update_bat(msg.percentage, msg.voltage)

    def status_sub(self, msg):
        self.data.update_state(msg.connected, msg.armed, msg.manual_input, msg.mode, msg.header.stamp.secs)

    def commanded_attitude_sub(self, msg):
        self.data.update_attitude_target(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w, msg.thrust)

    def estimator_type_sub(self, msg):
        self.data.update_estimator_type(msg.data)

    # main loop of ros node
    def run(self):
        while not rospy.is_shutdown():
            try:
                self.updateData.emit(0)
                self.rate.sleep()   ## sleep for 0.2 seconds
            except rospy.ROSInterruptException:
                print("ROS Shutdown Requested")
                break

class SingleDroneRosThread:
    def __init__(self, ui):
        super().__init__()
        self.rosQtObject = SingleDroneRosNode()
        self.thread = QThread()

        # setup signals
        self.ui = ui
        self.SetRosCallBack()

        # move and start thread
        self.rosQtObject.moveToThread(self.thread)
        self.lock = self.rosQtObject.data.lock
        self.thread.started.connect(self.rosQtObject.run)

        # set geofence
        self.ui.Geofence_X.display(self.rosQtObject.config['x'])
        self.ui.Geofence_Y.display(self.rosQtObject.config['y'])
        self.ui.Geofence_Z.display(self.rosQtObject.config['z'])

    def start(self):
        self.thread.start()

    # define the signal-slot combination of ros and pyqt GUI
    def SetRosCallBack(self):
        # feedbacks from ros
        self.rosQtObject.connectUpdateGUIData(self.UpdateGUIData)
        #self.rosQtObject.connectGUIPub(self.rosQtObject.publish_gui_state)

        # callbacks from GUI
        self.ui.SetHome.clicked.connect(self.send_set_home_request)
        self.ui.SimulationMode.stateChanged.connect(self.toggle_simulation_mode)
        self.ui.SendPositionUAV.clicked.connect(self.send_coordinates)
        self.ui.GetCurrentPositionUAV.clicked.connect(self.get_coordinates)

        self.ui.ARM.clicked.connect(lambda: self.send_arming_request(True, 0))
        self.ui.DISARM.clicked.connect(lambda: self.send_arming_request(False, 0))
        self.ui.Takeoff.clicked.connect(lambda: self.send_takeoff_request(float(self.ui.TakeoffHeight.text())))
        self.ui.Land.clicked.connect(lambda: self.send_land_request())
        self.ui.EmergencyStop.clicked.connect(lambda: self.send_arming_request(False, 21196))

        self.ui.OFFBOARD.clicked.connect(lambda: self.switch_mode("OFFBOARD"))
        self.ui.POSCTL.clicked.connect(lambda: self.switch_mode("POSCTL"))
        self.ui.HOLD.clicked.connect(self.hold)

    # update GUI data
    def UpdateGUIData(self):
        if not self.lock.tryLock():
            print("Could not lock the mutex")
            return
        imuMsg = self.rosQtObject.data.current_imu
        globalPosMsg = self.rosQtObject.data.current_global_pos
        localPosMsg = self.rosQtObject.data.current_local_pos
        velMsg = self.rosQtObject.data.current_vel

        batMsg = self.rosQtObject.data.current_battery_status
        stateMsg = self.rosQtObject.data.current_state
        attitudeTarg = self.rosQtObject.data.current_attitude_target
        indoor_mode = self.rosQtObject.data.indoor_mode
        self.lock.unlock()

        # accelerometer data
        self.ui.X_DISP.display("{:.2f}".format(imuMsg.roll, 2))
        self.ui.Y_DISP.display("{:.2f}".format(imuMsg.pitch, 2))
        self.ui.Z_DISP.display("{:.2f}".format(imuMsg.yaw, 2))

        self.ui.TargROLL_DISP.display("{:.2f}".format(attitudeTarg.roll, 2))
        self.ui.TargPITCH_DISP.display("{:.2f}".format(attitudeTarg.pitch, 2))
        self.ui.TargYAW_DISP.display("{:.2f}".format(attitudeTarg.yaw, 2))
        self.ui.TargTHRUST_DISP.display("{:.2f}".format(attitudeTarg.thrust, 2))

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

        # state updates
        self.ui.StateARM.setText("Armed" if stateMsg.armed else "Disarmed")
        self.ui.StateARM.setStyleSheet("color: red" if stateMsg.armed else "color: green")
        self.ui.StateConnected.setText("Connected" if stateMsg.connected else "Disconnected")
        self.ui.StateConnected.setStyleSheet("color: green" if stateMsg.connected else "color: red")
        self.ui.StateMode.setText(stateMsg.mode)

        if indoor_mode:
            self.ui.StateSimulation.setText("INDOOR")
            self.ui.StateSimulation.setStyleSheet("color: green")
            self.ui.SetHome.setEnabled(False)
        else:
            self.ui.StateSimulation.setText("OUTDOOR")
            self.ui.StateSimulation.setStyleSheet("color: orange")
            self.ui.SetHome.setEnabled(True)

        # misc data
        if batMsg: # takes long to initialize
            if self.ui.BatInd.isTextVisible() == False:
                self.ui.BatInd.setTextVisible(True)
            self.ui.BatInd.setValue(float(batMsg.percentage)*100)
            self.ui.VOLT_DISP.display("{:.2f}".format(batMsg.voltage, 2))

        # update seconds
        if not hasattr(self, 'seconds'):
            self.armed_seconds = 0
        if not hasattr(self, 'last_time'):
            self.last_time = stateMsg.seconds
        if stateMsg.armed:
            self.armed_seconds = stateMsg.seconds - self.last_time # time since armed
            self.ui.Sec_DISP.display("{}".format(self.armed_seconds, 1))
        else:
            self.last_time = stateMsg.seconds
            self.armed_seconds = 0
        # update minutes
        if self.armed_seconds == 60:
            self.ui.Min_DISP.display("{}".format(int(self.ui.Min_DISP.value() + 1), 1))
            self.last_time = stateMsg.seconds

    ### callback functions for modifying GUI elements ###
    def toggle_simulation_mode(self, state):
        if state == 2:
            print("Simulation controls available")
            self.ui.ARM.setEnabled(True)
            self.ui.DISARM.setEnabled(True)
            self.ui.Takeoff.setEnabled(True)
            self.ui.Land.setEnabled(True)
            self.ui.TakeoffHeight.setEnabled(True)
            self.ui.EmergencyStop.setEnabled(True)
            self.ui.OFFBOARD.setEnabled(True)
            self.ui.POSCTL.setEnabled(True)

        else:
            print("Simulation controls disabled")
            self.ui.ARM.setEnabled(False)
            self.ui.DISARM.setEnabled(False)
            self.ui.Takeoff.setEnabled(False)
            self.ui.Land.setEnabled(False)
            self.ui.TakeoffHeight.setEnabled(False)
            self.ui.EmergencyStop.setEnabled(False)
            self.ui.OFFBOARD.setEnabled(False)
            self.ui.POSCTL.setEnabled(False)
    
    def send_set_home_request(self):
        self.rosQtObject.set_home_override_service()
        home_position = CommandHomeRequest()
        home_position.latitude = self.rosQtObject.data.current_global_pos.latitude
        home_position.longitude = self.rosQtObject.data.current_global_pos.longitude
        home_position.altitude = self.rosQtObject.data.current_global_pos.latitude
        response = self.rosQtObject.set_home_service(home_position)
        print(response)

    def send_coordinates(self):
        # if text is inalid, warn user
        try :
            x = float(self.ui.XPositionUAV.text())
            y = float(self.ui.YPositionUAV.text())
            z = float(self.ui.ZPositionUAV.text())
            yaw = float(self.ui.YAWUAV.text())
        except ValueError:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Invalid input, make sure values are numbers")
            msg.setWindowTitle("Warning")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return
        
        # if values are not within 5 meters of current position warn user
        if abs(x) > int(self.rosQtObject.config['x']) or abs(y) > int(self.rosQtObject.config['x']) or abs(z) > int(self.rosQtObject.config['x']) or z <= 0:
            ## pop up dialog 
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("Position is not within 5 meters of current position")
            msg.setWindowTitle("Warning")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return

        self.rosQtObject.publish_coordinates(x, y, z, yaw)

    def get_coordinates(self):
        # get current relative position
        self.ui.XPositionUAV.setText("{:.2f}".format(self.rosQtObject.data.current_local_pos.x, 2))
        self.ui.YPositionUAV.setText("{:.2f}".format(self.rosQtObject.data.current_local_pos.y, 2))
        self.ui.ZPositionUAV.setText("{:.2f}".format(self.rosQtObject.data.current_local_pos.z, 2))
        self.ui.YAWUAV.setText("{:.2f}".format(self.rosQtObject.data.current_imu.yaw, 2))

    ### define publish / service functions to ros topics ###
    def send_arming_request(self, arm, param2):
        response = self.rosQtObject.arming_service(command=400, confirmation=0, param1 = arm, param2 = param2)
        print(response)
        return response

    def send_takeoff_request(self, req_altitude):
        arm_response = self.send_arming_request(True, 0)
        # if armed takeoff
        if arm_response.result == 0:
            self.rosQtObject.publish_coordinates(0, 0, req_altitude)
            print(f"Takeoff request sent at {req_altitude} meters")

    def send_land_request(self):
        response = self.rosQtObject.land_service(command=21, confirmation=0, param1 = 0, param7 = 0)
        self.rosQtObject.publish_coordinates(self.rosQtObject.data.current_local_pos.x, self.rosQtObject.data.current_local_pos.y, 0)
        print(response)

    def switch_mode(self, mode):
        response = self.rosQtObject.set_mode_service(custom_mode=mode)
        print(response)

    def hold(self):
        response = self.rosQtObject.publish_coordinates(self.rosQtObject.data.current_local_pos.x, self.rosQtObject.data.current_local_pos.y, 2, self.rosQtObject.data.current_imu.yaw)
        print(response)

