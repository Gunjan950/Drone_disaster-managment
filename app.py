import asyncio, json, math, os, threading
import paho.mqtt.client as mqtt
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

app = FastAPI()
state = {"humans": [], "drones": {}, "hazards": [], "brief": ""}
clients: set = set()
loop = None

def dist_m(a, b):
    return math.hypot((a["lat"]-b["lat"])*111320, (a["lon"]-b["lon"])*111320*math.cos(math.radians(a["lat"])))

def merge_human(evt):
    """Merge repeat sightings within 5 m into one pin (running average, keep best confidence)."""
    for h in state["humans"]:
        if dist_m(h, evt) < 5:
            n = h["sightings"]
            h["lat"] = (h["lat"]*n + evt["lat"])/(n+1); h["lon"] = (h["lon"]*n + evt["lon"])/(n+1)
            h["sightings"] = n+1; h["last_seen"] = evt["ts"]
            if evt["confidence"] > h["confidence"]:
                h["confidence"], h["thumb"] = evt["confidence"], evt.get("thumb")
            return h
    h = dict(id=len(state["humans"])+1, lat=evt["lat"], lon=evt["lon"], confidence=evt["confidence"],
             drone_id=evt["drone_id"], first_seen=evt["ts"], last_seen=evt["ts"], sightings=1,
             thumb=evt.get("thumb"), status="new")
    state["humans"].append(h); return h

async def broadcast(msg):
    for ws in list(clients):
        try: await ws.send_text(json.dumps(msg))
        except Exception: clients.discard(ws)

def on_message(_c, _u, m):
    try: data = json.loads(m.payload)
    except Exception: return
    if m.topic == "sar/detections" and data.get("event") == "human_detected":
        msg = {"type": "human", "data": merge_human(data)}
    elif m.topic.startswith("sar/telemetry/"):
        state["drones"][data["drone_id"]] = data; msg = {"type": "drone", "data": data}
    elif m.topic == "sar/hazards":
        state["hazards"].append(data); msg = {"type": "hazard", "data": data}
    elif m.topic == "sar/brief":
        state["brief"] = data.get("text", ""); msg = {"type": "brief", "data": state["brief"]}
    else: return
    if loop: asyncio.run_coroutine_threadsafe(broadcast(msg), loop)

@app.on_event("startup")
async def startup():
    global loop; loop = asyncio.get_running_loop()
    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2); cli.on_message = on_message
    cli.connect(os.getenv("MQTT_HOST", "localhost"), 1883); cli.subscribe([("sar/#", 0)])
    threading.Thread(target=cli.loop_forever, daemon=True).start()

@app.get("/api/state")
def get_state(): return state

@app.post("/api/human/{hid}/{status}")
async def set_status(hid: int, status: str):
    for h in state["humans"]:
        if h["id"] == hid:
            h["status"] = status; await broadcast({"type": "human", "data": h})
    return {"ok": True}

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept(); clients.add(ws)
    await ws.send_text(json.dumps({"type": "state", "data": state}))
    try:
        while True: await ws.receive_text()
    except WebSocketDisconnect: clients.discard(ws)

@app.get("/")
def index(): return FileResponse("static/index.html")
app.mount("/static", StaticFiles(directory="static"), name="static")
