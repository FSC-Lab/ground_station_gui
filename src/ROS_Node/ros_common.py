class IMUinfo:
    def __init__(self, ax=0, ay=0, az=0) -> None:
        self.ax = ax
        self.ay = ay
        self.az = az

class PositionInfo:
    def __init__(self, latitude=0, longitude=0, altitude=0) -> None:
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude