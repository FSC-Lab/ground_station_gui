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