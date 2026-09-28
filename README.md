# SAR Drone: detect survivors, geolocate, alert responders

## Quick start (no simulator needed)
```bash
docker compose up --build            # broker + dashboard (localhost:8000) + planner
cd edge && pip install -r requirements.txt
python edge_node.py --drone drone_1 --source video --path your_aerial_video.mp4 --lat 19.076 --lon 72.877
```
Open http://localhost:8000. Pins appear as people are detected.

## With AirSim
Copy `sim/settings.json` to `~/Documents/AirSim/settings.json`, start your Unreal map, then
`python edge_node.py --source airsim --vehicle Drone1_Wide --drone drone_1`.

## Layers
| Layer | Where | Rule |
|---|---|---|
| Flight safety (geofence, RTH, failsafe) | Pixhawk/PX4 | rule-based, never ML or LLM |
| Perception + geolocation | edge/ (Jetson) | 30+ FPS target: measure and report |
| Planning + LLM brief | planner/ (laptop) | LLM never in the control loop |

## Honest limits
- Sim "infrared" is not real thermal. Validate thermal on real datasets (e.g. FLIR ADAS, HIT-UAV).
- The stock YOLOv8 COCO model detects "person" only; fire/hazards need fine-tuning.
- Geolocation error: GPS 1-3 m + attitude/terrain error. Pass `range_m` from depth/LiDAR when available.
- Map tiles need internet; run a local tile server (or cache tiles) for offline field use.
- Real flights: DGCA Drone Rules 2021, authorized operators only.

## Next
`vehicle_interface` goto() implementations, hazard fine-tuning (training/), measured eval numbers.
