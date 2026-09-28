from abc import ABC, abstractmethod

class Vehicle(ABC):
    """One API, two backends. Planner/RL code only talks to this."""
    @abstractmethod
    def arm_takeoff(self, alt_m: float): ...
    @abstractmethod
    def goto(self, lat: float, lon: float, alt_m: float, speed_ms: float = 5.0): ...
    @abstractmethod
    def return_home(self): ...
    @abstractmethod
    def position(self) -> tuple: ...   # lat, lon, alt_agl

class AirSimVehicle(Vehicle):
    def __init__(self, name):
        import airsim; self.a = airsim; self.c = airsim.MultirotorClient(); self.c.confirmConnection()
        self.n = name; self.c.enableApiControl(True, name); self.c.armDisarm(True, name)
    def arm_takeoff(self, alt_m):
        self.c.takeoffAsync(vehicle_name=self.n).join(); self.c.moveToZAsync(-alt_m, 3, vehicle_name=self.n).join()
    def goto(self, lat, lon, alt_m, speed_ms=5.0):
        raise NotImplementedError("TODO: lat/lon -> NED offset from home, then moveToPositionAsync")
    def return_home(self): self.c.goHomeAsync(vehicle_name=self.n)
    def position(self):
        g = self.c.getGpsData(vehicle_name=self.n).gnss.geo_point; return g.latitude, g.longitude, g.altitude

class MavsdkVehicle(Vehicle):
    """PX4 (SITL or real Pixhawk) via MAVSDK-Python. Same interface, real hardware later."""
    def __init__(self, url="udp://:14540"): self.url = url
    def arm_takeoff(self, alt_m): raise NotImplementedError("TODO: mavsdk action.arm/takeoff")
    def goto(self, lat, lon, alt_m, speed_ms=5.0): raise NotImplementedError("TODO: action.goto_location")
    def return_home(self): raise NotImplementedError("TODO: action.return_to_launch")
    def position(self): raise NotImplementedError("TODO: telemetry.position")
