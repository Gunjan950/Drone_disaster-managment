"""Frame + telemetry sources. Same interface for sim, video file, and (later) real drone."""
import math, time, cv2

class VideoSource:
    """Video file or webcam with fixed telemetry - lets you demo without a simulator."""
    def __init__(self, path, lat, lon, alt=30.0, yaw=0.0, tilt_down=90.0, hfov=90.0):
        self.cap = cv2.VideoCapture(0 if path == "0" else path)
        self.tel = dict(lat=lat, lon=lon, alt_agl=alt, yaw=yaw, pitch=0.0, roll=0.0,
                        cam_tilt_down=tilt_down, hfov=hfov)
    def read(self):
        ok, frame = self.cap.read()
        return (frame, dict(self.tel, ts=time.time())) if ok else (None, None)

class AirSimSource:
    """Pulls RGB + pose from AirSim (pip install airsim). Lat/lon from getGpsData."""
    def __init__(self, vehicle, camera="rgb", cam_tilt_down=45.0, hfov=90.0):
        import airsim, numpy as np
        self.airsim, self.np = airsim, np
        self.c = airsim.MultirotorClient(); self.c.confirmConnection()
        self.v, self.cam, self.tilt, self.hfov = vehicle, camera, cam_tilt_down, hfov
    def read(self):
        a, np = self.airsim, self.np
        r = self.c.simGetImages([a.ImageRequest(self.cam, a.ImageType.Scene, False, False)], self.v)[0]
        frame = np.frombuffer(r.image_data_uint8, dtype=np.uint8).reshape(r.height, r.width, 3)
        gps = self.c.getGpsData(vehicle_name=self.v).gnss.geo_point
        st = self.c.getMultirotorState(vehicle_name=self.v)
        p, q = st.kinematics_estimated.position, st.kinematics_estimated.orientation
        pitch, roll, yaw = a.to_eularian_angles(q)
        return frame, dict(lat=gps.latitude, lon=gps.longitude, alt_agl=-p.z_val,
                           yaw=math.degrees(yaw), pitch=math.degrees(pitch), roll=math.degrees(roll),
                           cam_tilt_down=self.tilt, hfov=self.hfov, ts=time.time())
