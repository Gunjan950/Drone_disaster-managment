"""Ground-station planner: (1) split the AOI into non-overlapping sectors + lawnmower waypoints,
(2) turn detections into a plain-language responder brief (LLM optional, template fallback)."""
import json, math, os, time, urllib.request
import paho.mqtt.client as mqtt

AOI = dict(south=19.0740, west=72.8750, north=19.0780, east=72.8800)   # edit for your site
DRONES = {"drone_1": 50, "drone_2": 20, "drone_3": 8}                   # id -> altitude (m)
SWATH_M = {"drone_1": 60, "drone_2": 25, "drone_3": 10}                 # approx camera footprint width

def lawnmower(south, west, north, east, swath_m):
    step = swath_m / 111320; pts, lat, flip = [], south + step / 2, False
    while lat < north:
        row = [(lat, west), (lat, east)]; pts += row[::-1] if flip else row
        lat += step; flip = not flip
    return pts

def assign_sectors(aoi, drones):
    """Baseline: vertical strips, one per drone. The LLM may only review, never override safety."""
    w = (aoi["east"] - aoi["west"]) / len(drones); plans = {}
    for i, d in enumerate(drones):
        west = aoi["west"] + i * w
        plans[d] = dict(altitude_m=drones[d], sector=dict(aoi, west=west, east=west + w),
                        waypoints=lawnmower(aoi["south"], west, aoi["north"], west + w, SWATH_M[d]))
    return plans

def bearing_text(dlat, dlon):
    ang = (math.degrees(math.atan2(dlon, dlat)) + 360) % 360
    return ["north","north-east","east","south-east","south","south-west","west","north-west"][int((ang + 22.5) // 45) % 8]

def template_brief(humans, hazards):
    if not humans: return "No survivors located yet."
    lines = [f"{len(humans)} survivor(s) located."]
    for h in sorted(humans, key=lambda x: -x["confidence"]):
        lines.append(f"- #{h['id']} at {h['lat']:.5f}, {h['lon']:.5f} ({h['confidence']*100:.0f}% confidence)")
        for hz in hazards:
            d = math.hypot((hz["lat"]-h["lat"])*111320, (hz["lon"]-h["lon"])*111320)
            if d < 100: lines.append(f"  hazard: {hz.get('class','unknown')} {d:.0f} m {bearing_text(hz['lat']-h['lat'], hz['lon']-h['lon'])}")
    return "\n".join(lines)

def llm_brief(humans, hazards):
    """Optional: any OpenAI-compatible endpoint (Ollama, vLLM, llama.cpp). Falls back to the template."""
    url, model = os.getenv("LLM_URL"), os.getenv("LLM_MODEL")
    if not url or not model: return template_brief(humans, hazards)
    try:
        body = json.dumps(dict(model=model, messages=[
            {"role": "system", "content": "Write a short factual brief for disaster responders. Use only the data given. Never invent locations, counts or hazards."},
            {"role": "user", "content": json.dumps(dict(survivors=humans, hazards=hazards))}])).encode()
        req = urllib.request.Request(url + "/chat/completions", body, {"Content-Type": "application/json"})
        return json.load(urllib.request.urlopen(req, timeout=20))["choices"][0]["message"]["content"]
    except Exception:
        return template_brief(humans, hazards)

def main():
    dets, hazards, cli = [], [], mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    def on_msg(_c, _u, m):
        d = json.loads(m.payload)
        (dets if m.topic == "sar/detections" else hazards).append(d)
    cli.on_message = on_msg
    cli.connect(os.getenv("MQTT_HOST", "localhost"), 1883)
    cli.subscribe([("sar/detections", 1), ("sar/hazards", 0)]); cli.loop_start()
    for d, p in assign_sectors(AOI, DRONES).items():
        cli.publish(f"sar/plan/{d}", json.dumps(p), retain=True)
    print("Plans published."); last = 0
    while True:
        time.sleep(5)
        if len(dets) != last:
            last = len(dets)
            hs = [dict(id=i, lat=d["lat"], lon=d["lon"], confidence=d["confidence"]) for i, d in enumerate(dets, 1)]
            cli.publish("sar/brief", json.dumps({"text": llm_brief(hs, hazards)}))

if __name__ == "__main__":
    main()
