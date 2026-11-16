"""Run your fine-tuned Isaac 0.5 deployment on the SO-101 follower (simple mode: predict a chunk, play it, repeat).

Built from the Perceptron Isaac 0.5 workshop slides + robotics.perceptron.inc/docs.

Usage (conda env active, leader arm NOT needed):
  export TEAM_API_KEY=...                      # never typed into the command itself
  python run_policy.py --deployment dep_xxx --dry-run      # 1) safe test: talks to the model, does NOT move the arm
  python run_policy.py --deployment dep_xxx                # 2) real run (hand near the power plug!)

Ctrl+C stops cleanly. NOTE: the follower goes limp when it disconnects (torque off) - support the arm.
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import requests

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

CONTROL = "https://robotics.perceptron.inc/v1"
INFER = CONTROL + "/inference"
JOINTS = ["shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos",
          "wrist_flex.pos", "wrist_roll.pos", "gripper.pos"]  # contract order
UA = {"User-Agent": "battery-catcher-dubhacks/1.0"}  # slides: some edge rules reject default UAs
HZ = 30


def die(msg, r=None):
    sys.exit(f"ERROR: {msg}" + (f"\n{r.status_code}: {r.text[:1500]}" if r is not None else ""))


def png(rgb: np.ndarray) -> bytes:
    ok, buf = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    if not ok:
        die("PNG encode failed")
    return buf.tobytes()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--deployment", required=True, help="deployment id in state 'ready'")
    ap.add_argument("--instruction", default="sort the trash", help="EXACT task text you recorded with")
    ap.add_argument("--port", default="/dev/tty.usbmodem5C821089511")
    ap.add_argument("--robot-id", default="follow_robot")
    ap.add_argument("--side", type=int, default=1, help="side camera index")
    ap.add_argument("--wrist", type=int, default=0, help="wrist camera index")
    ap.add_argument("--session-id", default=f"robot-{int(time.time())}")
    ap.add_argument("--seconds", type=float, default=60, help="stop after this many seconds")
    ap.add_argument("--max-step", type=float, default=8.0,
                    help="safety: max change per 33 ms tick (degrees / gripper units)")
    ap.add_argument("--deadline-ms", type=float, default=400.0)
    ap.add_argument("--rows", type=int, default=30, help="rows of each chunk to play before re-planning (<=30)")
    ap.add_argument("--dry-run", action="store_true", help="query the model but never move the arm")
    a = ap.parse_args()

    key = os.environ.get("TEAM_API_KEY") or die("export TEAM_API_KEY first")

    # 1. Grant: trade the team key for session credentials (team key goes ONLY to the control API).
    r = requests.post(f"{CONTROL}/deployments/{a.deployment}/sessions",
                      headers={**UA, "Authorization": f"Bearer {key}"},
                      json={"session_id": a.session_id, "ttl_seconds": 21600}, timeout=60)
    if r.status_code >= 300:
        die("session grant failed", r)
    g = r.json()
    sid = g.get("session_id", a.session_id)
    H = {**UA, "Authorization": f"Bearer {g['application_session_token']}",
         "Modal-Authorization": f"Bearer {g['modal_session_token']}"}
    del key  # never sent to the inference API

    # 2. Serving contract.
    r = requests.get(f"{INFER}/deployments/{a.deployment}/contract", headers=H, timeout=60)
    if r.status_code >= 300:
        die("contract fetch failed", r)
    contract = r.json()
    cid = contract["contract_id"]
    print("Contract:", json.dumps(contract, indent=1)[:1500])

    # 3. Robot session (one instruction per session).
    r = requests.post(f"{INFER}/deployments/{a.deployment}/sessions", headers=H,
                      json={"instruction": a.instruction, "rtc_enabled": True, "contract_id": cid}, timeout=60)
    if r.status_code >= 300:
        die("robot session create failed", r)
    print(f"Session {sid} open. Instruction: {a.instruction!r}")

    # 4. Robot: same port, id (=calibration) and cameras as recording.
    cams = {"side": OpenCVCameraConfig(index_or_path=a.side, width=640, height=480, fps=30),
            "wrist": OpenCVCameraConfig(index_or_path=a.wrist, width=640, height=480, fps=30)}
    robot = SO101Follower(SO101FollowerConfig(port=a.port, id=a.robot_id, cameras=cams, use_degrees=True,
                                              max_relative_target=a.max_step))
    robot.connect()
    print("Robot connected." + ("  DRY RUN: arm will not move." if a.dry_run else ""))

    seq, t0, chunks = 0, time.monotonic(), 0
    try:
        while time.monotonic() - t0 < a.seconds:
            obs = robot.get_observation()
            cap = time.monotonic()
            state = [float(obs[j]) for j in JOINTS]
            meta = {"contract_id": cid, "observation_id": f"obs_{seq:06d}", "sequence": seq,
                    "timestamp_seconds": round(cap - t0 + 1.0, 4), "state": state,
                    "image_encoding": "png", "deadline_ms": a.deadline_ms}
            files = {"side": ("side.png", png(obs["side"]), "image/png"),
                     "wrist": ("wrist.png", png(obs["wrist"]), "image/png")}
            meta["capture_age_ms"] = round((time.monotonic() - cap) * 1000, 1)
            r = requests.post(f"{INFER}/sessions/{sid}/predict", headers=H,
                              data={"metadata": json.dumps(meta)}, files=files, timeout=30)
            seq += 1
            if r.status_code >= 300:
                print(f"predict {r.status_code}: {r.text[:500]}  (skipping; arm holds)")
                time.sleep(0.2)
                continue
            out = r.json()
            acts = np.asarray(out.get("actions", []), dtype=float)
            start = int(out.get("execute_from_index", 0))
            if acts.ndim != 2 or acts.shape[1] != 6 or not np.isfinite(acts).all():
                print("Malformed chunk, not applying:", str(out)[:300])
                continue
            chunks += 1
            rtt = (time.monotonic() - cap) * 1000
            print(f"chunk {chunks}: {len(acts)} rows, rtt {rtt:.0f} ms, timings {out.get('timings')}")
            # Play rows at 30 Hz (actions are already in robot units: never rescale).
            for row in acts[start:start + a.rows]:
                tick = time.monotonic()
                if not a.dry_run:
                    robot.send_action({j: float(v) for j, v in zip(JOINTS, row)})
                time.sleep(max(0.0, 1 / HZ - (time.monotonic() - tick)))
            if a.dry_run:
                print("   first row:", np.round(acts[start], 1).tolist(), " current:", np.round(state, 1).tolist())
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        try:
            requests.delete(f"{INFER}/sessions/{sid}", headers=H, timeout=10)
        except Exception:
            pass
        robot.disconnect()
        print(f"Done: {chunks} chunks. Session closed.")


if __name__ == "__main__":
    main()
