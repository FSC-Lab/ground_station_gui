class IMUinfo:
    def __init__(self, x=0, y=0, z=0) -> None:
        self.roll = x
        self.pitch = y
        self.yaw = z

class GlobalPositionInfo:
    def __init__(self, latitude=0, longitude=0, altitude=0) -> None:
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

class LocalPositionInfo:
    def __init__(self, x=0, y=0, z=0) -> None:
        self.x = x
        self.y = y
        self.z = z
    
class VelocityInfo:
    def __init__(self, vx=0, vy=0, vz=0) -> None:
        self.vx = vx
        self.vy = vy
        self.vz = vz

class BatteryInfo:
    def __init__(self, percentage=0, voltage=0) -> None:
        self.percentage = percentage
        self.voltage = voltage

class StateInfo:
    def __init__(self, connected=False, armed=False, manual_input=False, mode="", seconds=0) -> None:
        self.connected = connected
        self.armed = armed
        self.manual_input = manual_input
        self.mode = mode
        self.seconds = seconds
        self.total_seconds = 0
        
class GUIState:
    def __init__(self, armed=False, takeoff=False, land=False, altitude=0, e_stop=False) -> None:
        self.arm_pressed = armed
        self.takeoff_pressed = takeoff
        self.takeoff_altitude = altitude
        self.land_pressed = land
        self.e_stop_pressed = e_stop
        


