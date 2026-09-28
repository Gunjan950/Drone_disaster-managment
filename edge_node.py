"""Edge perception node: detect -> geolocate -> publish over MQTT.
  python edge_node.py --drone drone_1 --source video --path demo.mp4 --lat 19.07 --lon 72.87
  python edge_node.py --drone drone_1 --source airsim --vehicle Drone1_Wide
"""
import argparse, base64, json, os, queue, threading, time
import cv2, paho.mqtt.client as mqtt
from ultralytics import YOLO
from geolocate import pixel_to_gps
from sources import VideoSource, AirSimSource

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drone", default="drone_1")
    ap.add_argument("--source", choices=["video", "airsim"], default="video")
    ap.add_argument("--path", default="0"); ap.add_argument("--vehicle", default="Drone1_Wide")
    ap.add_argument("--lat", type=float, default=19.0760); ap.add_argument("--lon", type=float, default=72.8777)
    ap.add_argument("--model", default="yolov8n.pt")      # for speed: export to TensorRT -> yolov8n.engine
    ap.add_argument("--conf", type=float, default=0.4); ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--mqtt", default=os.getenv("MQTT_HOST", "localhost"))
    a = ap.parse_args()

    src = VideoSource(a.path, a.lat, a.lon) if a.source == "video" else AirSimSource(a.vehicle)
    model = YOLO(a.model)
    person_ids = [i for i, n in model.names.items() if n in ("person", "human")]

    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2); cli.connect(a.mqtt, 1883); cli.loop_start()
    out_q = queue.Queue(maxsize=64)          # publishing happens off the inference thread
    def pump():
        while True: cli.publish(*out_q.get())
    threading.Thread(target=pump, daemon=True).start()

    last_pub, frames, t0 = {}, 0, time.time()
    while True:
        frame, tel = src.read()
        if frame is None: break
        res = model.track(frame, persist=True, conf=a.conf, imgsz=a.imgsz, classes=person_ids, verbose=False)[0]
        h, w = frame.shape[:2]
        for b in res.boxes:
            tid = int(b.id) if b.id is not None else -1
            if time.time() - last_pub.get(tid, 0) < 2.0: continue     # 1 alert / 2 s per track
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            geo = pixel_to_gps((x1 + x2) / 2, y2, w, h, tel["hfov"], tel["lat"], tel["lon"], tel["alt_agl"],
                               tel["yaw"], tel["pitch"], tel["roll"], tel["cam_tilt_down"])  # bottom-centre = ground contact
            if geo is None: continue
            last_pub[tid] = time.time()
            crop = frame[max(int(y1), 0):int(y2), max(int(x1), 0):int(x2)]
            ok, jpg = cv2.imencode(".jpg", cv2.resize(crop, (96, 128))) if crop.size else (False, None)
            evt = dict(event="human_detected", drone_id=a.drone, track_id=tid, ts=tel["ts"],
                       lat=geo[0], lon=geo[1], confidence=round(float(b.conf), 3), modality=["rgb"],
                       thumb=base64.b64encode(jpg).decode() if ok else None)
            try: out_q.put_nowait(("sar/detections", json.dumps(evt), 1))
            except queue.Full: pass
        frames += 1
        if frames % 30 == 0:
            fps = frames / (time.time() - t0)
            try: out_q.put_nowait((f"sar/telemetry/{a.drone}", json.dumps(dict(tel, drone_id=a.drone, fps=round(fps, 1))), 0))
            except queue.Full: pass
            print(f"{fps:.1f} FPS")   # report the number you actually measure

if __name__ == "__main__":
    main()
