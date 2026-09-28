"""Pixel -> GPS via ray-casting to a ground plane (or a measured range).
Frames: camera x=right,y=down,z=forward; body FRD; world NED."""
import numpy as np

M_PER_DEG_LAT = 111_320.0

def _Rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def pixel_to_gps(u, v, img_w, img_h, hfov_deg, lat, lon, alt_agl_m,
                 yaw_deg, pitch_deg, roll_deg, cam_tilt_down_deg=0.0,
                 range_m=None):
    """Return (lat, lon, ground_distance_m) or None.
    alt_agl_m: height above ground (use terrain-corrected value if known).
    range_m: optional measured distance along the ray (LiDAR/depth) - more
             accurate than the flat-ground assumption at low altitude."""
    fx = (img_w / 2) / np.tan(np.radians(hfov_deg) / 2)
    ray_cam = np.array([(u - img_w / 2) / fx, (v - img_h / 2) / fx, 1.0])
    ray_cam /= np.linalg.norm(ray_cam)
    ray_body = _Ry(-np.radians(cam_tilt_down_deg)) @ np.array([ray_cam[2], ray_cam[0], ray_cam[1]])
    R = _Rz(np.radians(yaw_deg)) @ _Ry(np.radians(pitch_deg)) @ _Rx(np.radians(roll_deg))
    ray = R @ ray_body                      # NED
    if range_m is not None:
        t = range_m
    else:
        if ray[2] <= 1e-3:                  # at/above horizon: no ground hit
            return None
        t = alt_agl_m / ray[2]
    north, east = ray[0] * t, ray[1] * t
    out_lat = lat + north / M_PER_DEG_LAT
    out_lon = lon + east / (M_PER_DEG_LAT * np.cos(np.radians(lat)))
    return out_lat, out_lon, float(np.hypot(north, east))

if __name__ == "__main__":  # camera straight down, centre pixel -> directly under drone
    print(pixel_to_gps(320, 240, 640, 480, 90, 19.0, 72.8, 30, 0, 0, 0, 90))
