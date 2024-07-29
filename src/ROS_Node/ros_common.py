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

class Vector3:
    def __init__(self, x=0, y=0, z=0) -> None:
        self.x = x
        self.y = y
        self.z = z

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

class AttitudeTarget:
    def __init__(self, roll=0, pitch=0, yaw=0, thrust=0) -> None:
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.thrust = thrust
