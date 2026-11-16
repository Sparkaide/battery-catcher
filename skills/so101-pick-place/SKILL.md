---
name: so101-pick-place
description: Drive the trash-robot SO-101 follower arm directly (no trained policy) to pick up AA batteries seen by the side/wrist cameras and drop them in the cardboard box. Use when asked to control the arm, pick up batteries, or put things in the box.
---

# SO-101 battery pick-and-place (direct control from a Claude session)

Proven on 2026-10-17: battery "C" (on the black cloth) was picked and dropped in the box in about 15 minutes, after about 20 minutes of exploration. This file captures everything learned so the next run can skip the exploration.

## 0. How control works (read first)
- The Claude device shell (`device_bash`) is a Linux sandbox. It can **not** see the Mac's USB serial port or cameras.
- Computer use gives Terminal only "click" tier, so Claude cannot type commands there.
- So the human runs **`claude_bridge.py`** (in the trash-robot folder root) in Terminal:
  ```bash
  conda activate lerobot && python ~/trash-robot/claude_bridge.py
  # optional args: [PORT] [ROBOT_ID]; defaults /dev/tty.usbmodem5C821089511 follow_robot
  ```
  The bridge watches `trash-robot/bridge/cmd/*.json`, executes only robot ops, and writes `bridge/resp/<id>.json` plus snapshots in `bridge/snaps/`. It writes `bridge/READY` when connected; check `bridge/log.txt` for errors.
- Before asking the human to start it, tell them to quit lerobot-teleoperate, lerobot-record, Rerun and camera apps, and to leave the arm in its rest pose. One Ctrl+C makes the arm return slowly to its start pose and release torque. A second Ctrl+C kills it immediately and the arm drops.
- Bridge safety built in:
  - Goal is synced to present position before torque enables (no jump).
  - Smooth cosine interpolation, capped at 30 deg/s.
  - At most 60 deg change per joint per command (split bigger moves).
  - Joints are clamped to calibrated limits.

### Bridge ops (JSON)
| op | fields | notes |
|---|---|---|
| `probe_cams` | `max_idx` | saves `snaps/probe_<i>.jpg` |
| `set_cams` | `cams: {"side":1,"wrist":0}` | opens cameras persistently |
| `move` | `target: {joint: deg}`, `duration` | absolute; joints `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` |
| `nudge` | `delta: {joint: deg}` | relative |
| `rest` | | back to the start pose |
| `state` | | |
| `shutdown` | | rest, then disconnect |

Every op accepts `"snap": false` to skip snapshots (faster for intermediate waypoints).

### Helper scripts (`scripts/`; run them with device_bash at `$HOME/mnt/trash-robot/skills/so101-pick-place/scripts`)
- `do.py '<json>' [--crop x0,y0,x1,y1]` sends a command and waits. It builds `bridge/view/<tag>.jpg` (side | wrist side by side, yellow ticks every 80 px). With `--crop` it also builds `bridge/view/z.jpg` (2.5× zoom of a side-camera region | wrist).
- `cart.py X Z [PAN|-] [EXTRA_JSON] [--a3 -90] [--dur 2] [--nosnap]` prints a `move` for a Cartesian wrist target. It includes elbow gravity compensation. Typical use:
  `python3 do.py "$(python3 cart.py 0.078 0.045 -20 '{"gripper":85}')" --crop 280,100,520,280`
- `ik.py` holds the kinematic model.
- To view an image: `device_stage_files` on `/Users/aquar/trash-robot/bridge/view/<file>.jpg`, then `Read` the staged path. One combined image per step halves the round trips.

## 1. Hardware facts (this arm)
- Uses lerobot 0.6.2. The `so101_follower` config is found through `RobotConfig.get_choice_class` (the bridge handles the import paths).
- Cameras (OpenCV/AVFoundation indexes, which may change after replugging; run `probe_cams` to recheck):
  - **side = 1**
  - **wrist = 0**
  - 2 = MacBook webcam
  - 3 = black/virtual
- Start (rest) pose:
  ```
  pan 9.4, lift -104.1, elbow 97.1, wrist_flex 63.1, roll 19.7, gripper 42.2
  ```
  The rest pose sits slightly outside the computed limits (lift ±102.7, elbow ±95.5). That's harmless.
- Limits (degrees):
  - pan ±111
  - lift ±102.7
  - elbow ±95.5
  - wrist_flex ±101 (stay ≤ 99)
  - roll ±178
  - gripper 0–100
- **Gripper:** higher = open.
  - ~41.6 = fully closed with nothing held.
  - Use 75–85 to open, 85 for a wide approach.
  - Command 30 to close (it stalls on the object).
  - Holding an AA by the fingertips reads ~43. A reading of 41.6 after closing means a miss.
- **Elbow sags ~+4°** under gravity (it ends ~4° more folded than commanded). `cart.py` subtracts 3.5.
- **Contact detection:** when the fingertip rests on the table, the elbow sag disappears (measured elbow ≈ commanded). Also shoulder_lift ends ~1° short and wrist_flex ~1.4° over. Use this to confirm the tip is down.

## 2. Kinematic model (`ik.py`)
- L1 = 0.116 m, L2 = 0.135 m.
- With the gripper pointing straight down (a3 = -90), the **fixed-jaw tip touches the cloth at wrist z ≈ 0.012–0.02**. z is measured from the shoulder-lift axis.
- Hover heights (wrist z):
  - 0.045: tip a few cm up
  - 0.10: about 10 cm up
  - 0.13: clears the box walls with a battery hanging from the tips
- Reach limit: L1 + L2 = 0.251. With a3 = -90, wrist_flex exceeds 99 when z ≳ 0.12 at x ≤ 0.10, or z ≳ 0.13 at larger x. For far or high poses, tilt the gripper with `--a3 -75` or `--a3 -60`.
- Model accuracy is about 1 cm. Always close the loop with the cameras.

## 3. Reading the cameras
- **Side camera:** looks at the table from the front.
  - The cardboard **box** is at the top-centre (≈ x 160–440, y 15–110).
  - The **arm base** is just off-frame at the top-right.
  - At pan ≈ 0–20, increasing reach (x) moves the gripper **left** in the side image (toward the box). Pan + moves it away from the camera (up in the image).
  - The camera was bumped once mid-session (the scene shifted ~20 px), so don't hard-code pixel targets.
- **Depth trick that works:** zoom on the side camera. If the fixed jaw **occludes the battery**, the jaw is in front of it (closer to the camera). If the battery occludes the jaw, the jaw is behind it. The jaw's shadow offset shows how high the tip is.
- **Wrist camera:** mounted on the fixed jaw, looking down along the fingers.
  - The fixed finger is at the bottom-centre (tip ≈ x 275–300, y 330).
  - The moving jaw is on the **right** and swings out of frame when open (at 58 it sits at x ≈ 505–580).
  - **The jaws close horizontally in the image**, so the battery's long axis must look **vertical** in the wrist image, and the battery must sit **right of the fixed finger**.
- **Wrist-image directions at roll ≈ -107:**
  - pan − (−1°) moves the content right by ~7–9 px at hover height (more when lower).
  - Reach + (+1 cm) moves the content down by ~20 px at z 0.10.
  - Rolling the wrist by −60° rotated the content counter-clockwise by about 50–60°.
  - Rolling changes these mappings, so re-measure with one small nudge after any big roll change.
- **Descending changes the picture:** objects at table level drift toward the **bottom** of the wrist image. When the tips are above the table, the spot under the tips appears below the fingertip pixel. As the tips reach the table it converges to the fingertip line (y ≈ 330).

## 4. Grasp recipe (worked)
1. Hover over the target with the gripper vertical, at z ≈ 0.10 or so. Find the battery in the wrist view.
2. Set `wrist_roll` so the battery is vertical in the wrist image. For battery C this was **roll ≈ -107** (from 19.7). Split the move to stay under 60° per command.
3. Lower to z 0.045 with the gripper at 85. Shift pan and reach until the battery is right of the fixed finger and straddles the fingertip line (y ≈ 260–410).
4. **Push the fixed (stationary) jaw tip all the way down onto the cloth.** This is the operator's key advice. The first attempt at z 0.025 missed because the jaws passed over the battery. The successful grasp commanded **z = 0.008**, pressing the tip into the cloth. Confirm contact from the joint errors (section 1).
5. Close to gripper 30 over 2.5 s. Expect a reading of ~43 if the battery is held.
6. Lift to z 0.045 and check the wrist view: the battery should be between the fingertips (it hangs end-on).
7. If the miss reading appears (41.6), reopen at z 0.04. The battery has usually been pushed ~1 cm toward the fixed jaw, so recentre and repeat with the tip pressed down.

Successful grasp pose for C:
```
pan -19.5, cart.py 0.078 0.008, roll -107, gripper 30
```
Before it, the 0.095 hover was ~2 cm too far out.

## 5. Transport and drop in the box (worked)
Waypoints (keep the gripper at 30 throughout):
1. `cart 0.09 0.08`
2. `cart 0.12 0.13`
3. `cart 0.14 0.13 pan 15`
4. `cart 0.18 0.13 pan 22 --a3 -75`
5. Final drop pose: **`cart 0.20 0.11 pan 38 --a3 -60`**. It measured:
   ```
   pan 36.8, lift 35, elbow -44, wrist 67.6, roll -107
   ```
   Then gripper 75. The battery landed on the box floor.

Use 3 s durations for the big transfer moves so the battery doesn't swing. Before releasing, the wrist view should show box floor under and around the tips, with the box's left wall edge to the left of the fingers.

## 6. Scene at the end of the session (it may have changed)
| Battery | Location | Status |
|---|---|---|
| C | on the black cloth, side ≈ (410,208) | **in the box** |
| A | on the white paper just in front of the box, side ≈ (315,130) | remaining |
| B | on the cloth right of the paper, close to the arm base, side ≈ (485,112) | remaining |
| D | on the silver bag, lower right, side ≈ (510–580, 280–300) | remaining |

Distractors: crumpled paper, a silver anti-static bag, and white paper. Leave them alone. The arm was left hovering over the box with the gripper open (pan 36.8). Send `rest` or `shutdown` when done.

## 7. Efficiency tips for the next agent
- Start with `probe_cams`, then `set_cams`, then one exploratory raise to `cart 0.06 0.08`. After that you can go straight to the target.
- Batch waypoints with `"snap": false` and only snapshot at decision points.
- Each look costs about 3 tool calls (send, stage, read). Use `--crop` around the gripper for depth judgments.
- B is very close to the base. It may need a small reach (x ≈ 0.04–0.06) and the gripper tilted (`--a3`) because of the wrist_flex limit.
