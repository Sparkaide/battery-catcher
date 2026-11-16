#!/usr/bin/env python
"""
Claude <-> SO-101 bridge.
Run in Terminal:  conda activate lerobot && python ~/trash-robot/claude_bridge.py
It only executes robot commands (move / gripper / snapshot) written as JSON files
into ./bridge/cmd/. It never runs shell commands.
Ctrl+C once  -> arm returns slowly to its starting pose, torque released, exit.
"""
import json, time, sys, traceback, pathlib, signal, math, importlib, glob, os

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
B = ROOT / "bridge"; CMD = B / "cmd"; RESP = B / "resp"; SNAP = B / "snaps"
for d in (CMD, RESP, SNAP):
    d.mkdir(parents=True, exist_ok=True)
LOG = open(B / "log.txt", "a", buffering=1)

def log(*a):
    s = time.strftime("%H:%M:%S ") + " ".join(str(x) for x in a)
    print(s, flush=True); LOG.write(s + "\n")

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbmodem5C821089511"
ROBOT_ID = sys.argv[2] if len(sys.argv) > 2 else "follow_robot"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
MAX_SPEED = 30.0   # deg/s (gripper units/s) cap
MAX_STEP_CMD = 60.0  # max change per joint per single command
STOP = {"flag": False}

robot = None
limits = {}
start_pose = None
cams = {}

# ---------------- robot ----------------
def make_robot():
    import lerobot
    log("lerobot version:", getattr(lerobot, "__version__", "?"))
    for m in ["lerobot.robots.so101_follower", "lerobot.robots.so_follower",
              "lerobot.common.robots.so101_follower"]:
        try: importlib.import_module(m)
        except Exception as e: log("import", m, "->", type(e).__name__)
    RobotConfig = make = None
    for m in ["lerobot.robots", "lerobot.common.robots"]:
        try:
            mod = importlib.import_module(m)
            RobotConfig = mod.RobotConfig; make = mod.make_robot_from_config; break
        except Exception as e: log("import", m, "->", e)
    cls = RobotConfig.get_choice_class("so101_follower")
    import dataclasses
    fields = {f.name for f in dataclasses.fields(cls)}
    kw = dict(port=PORT, id=ROBOT_ID)
    if "use_degrees" in fields: kw["use_degrees"] = True
    if "cameras" in fields: kw["cameras"] = {}
    cfg = cls(**kw)
    r = make(cfg)
    # Before torque turns on, set goal = present position so the arm cannot jump.
    bus = r.bus
    orig = bus.enable_torque
    def safe_enable(*a, **k):
        try:
            pos = bus.sync_read("Present_Position", normalize=False)
            bus.sync_write("Goal_Position", pos, normalize=False)
            log("goal synced to present before torque enable")
        except Exception as e:
            log("WARN could not sync goal:", e)
        return orig(*a, **k)
    bus.enable_torque = safe_enable
    r.connect()
    return r

def read_state():
    obs = robot.bus.sync_read("Present_Position")
    return {j: round(float(obs[j]), 2) for j in JOINTS}

def compute_limits():
    out = {}
    for j in JOINTS:
        c = robot.bus.calibration[j]
        if j == "gripper":
            out[j] = (0.0, 100.0)
        else:
            res = 4095
            half = (c.range_max - c.range_min) / 2 * 360 / res
            out[j] = (-half + 2, half - 2)
    return out

def clamp(j, v):
    lo, hi = limits[j]
    return min(max(v, lo), hi)

def goto(target, duration=None, check_stop=True):
    cur = read_state()
    tgt = dict(cur)
    for j, v in target.items():
        if j not in JOINTS: raise ValueError(f"unknown joint {j}")
        v = float(v)
        if abs(v - cur[j]) > MAX_STEP_CMD:
            raise ValueError(f"{j}: change {v-cur[j]:.1f} exceeds {MAX_STEP_CMD} per command")
        tgt[j] = clamp(j, v)
    dmax = max(abs(tgt[j] - cur[j]) for j in JOINTS)
    dmin_needed = dmax / MAX_SPEED
    duration = max(float(duration or 0), dmin_needed, 0.3)
    n = max(int(duration * 50), 1)
    t0 = time.time()
    for i in range(1, n + 1):
        if check_stop and STOP["flag"]: break
        s = i / n; s = 0.5 - 0.5 * math.cos(math.pi * s)
        act = {f"{j}.pos": cur[j] + (tgt[j] - cur[j]) * s for j in JOINTS}
        robot.send_action(act)
        dt = t0 + i * duration / n - time.time()
        if dt > 0: time.sleep(dt)
    time.sleep(0.4)
    return tgt

# ---------------- cameras ----------------
def open_cam(idx):
    cap = cv2.VideoCapture(int(idx), cv2.CAP_AVFOUNDATION) if sys.platform == "darwin" else cv2.VideoCapture(int(idx))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    for _ in range(15): cap.read()
    return cap

def snap(tag):
    paths = {}
    for name, cap in cams.items():
        frame = None
        for _ in range(6):
            ok, f = cap.read()
            if ok: frame = f
        if frame is None: paths[name] = "READ FAILED"; continue
        p = SNAP / f"{tag}_{name}.jpg"
        cv2.imwrite(str(p), frame); paths[name] = p.name
    return paths

def probe(max_idx=6):
    res = {}
    for i in range(max_idx):
        try:
            cap = open_cam(i)
            ok, f = cap.read()
            if ok and f is not None:
                p = SNAP / f"probe_{i}.jpg"; cv2.imwrite(str(p), f)
                res[i] = {"shape": list(f.shape), "file": p.name}
            else:
                res[i] = "no frame"
            cap.release()
        except Exception as e:
            res[i] = str(e)
    return res

# ---------------- command loop ----------------
def handle(c):
    global start_pose
    op = c.get("op")
    tag = c.get("_id", "x")
    out = {}
    if op == "ping":
        pass
    elif op == "probe_cams":
        out["probe"] = probe(c.get("max_idx", 6))
    elif op == "set_cams":
        for cap in cams.values(): cap.release()
        cams.clear()
        for name, idx in c["cams"].items(): cams[name] = open_cam(idx)
    elif op == "move":
        goto(c["target"], c.get("duration"))
    elif op == "nudge":
        cur = read_state()
        goto({j: cur[j] + float(d) for j, d in c["delta"].items()}, c.get("duration"))
    elif op == "rest":
        goto(start_pose, c.get("duration", 4))
    elif op == "state":
        pass
    elif op == "shutdown":
        STOP["flag"] = True
    else:
        raise ValueError(f"unknown op {op}")
    out["state"] = read_state()
    out["limits"] = limits
    out["start_pose"] = start_pose
    if c.get("snap", True) and cams:
        out["snaps"] = snap(tag)
    return out

def on_sigint(sig, frm):
    if STOP["flag"]:
        log("second Ctrl+C: exiting immediately"); os._exit(1)
    STOP["flag"] = True
    log("Ctrl+C: stopping, returning to start pose...")
signal.signal(signal.SIGINT, on_sigint)

def main():
    global robot, limits, start_pose
    for f in glob.glob(str(CMD / "*.json")): os.remove(f)   # discard stale commands
    robot = make_robot()
    limits = compute_limits()
    start_pose = read_state()
    log("connected. start pose:", start_pose)
    log("limits:", limits)
    (B / "READY").write_text(json.dumps({"start_pose": start_pose, "limits": limits, "t": time.time()}))
    while not STOP["flag"]:
        files = sorted(CMD.glob("*.json"))
        if not files: time.sleep(0.05); continue
        f = files[0]
        try:
            c = json.loads(f.read_text())
        except Exception:
            time.sleep(0.05); continue
        cid = f.stem; c["_id"] = cid; f.unlink()
        log("cmd", cid, json.dumps({k: v for k, v in c.items() if k != "_id"}))
        try:
            out = handle(c); out["ok"] = True
        except Exception as e:
            out = {"ok": False, "error": f"{type(e).__name__}: {e}", "tb": traceback.format_exc()}
            try: out["state"] = read_state()
            except Exception: pass
        tmp = RESP / f"{cid}.tmp"; tmp.write_text(json.dumps(out, indent=1)); tmp.rename(RESP / f"{cid}.json")
        log("done", cid, "ok" if out["ok"] else out["error"], out.get("state"))
    # shutdown
    try:
        STOP["flag"] = False
        log("returning to start pose", start_pose)
        goto(start_pose, 4, check_stop=False)
    except Exception as e:
        log("could not return to start:", e)
    (B / "READY").unlink(missing_ok=True)
    robot.disconnect()
    for cap in cams.values(): cap.release()
    log("disconnected, bye")

if __name__ == "__main__":
    try:
        main()
    except Exception:
        log("FATAL\n" + traceback.format_exc())
        (B / "READY").unlink(missing_ok=True)
