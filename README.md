
# Battery Catcher
Battery-sorting robot arm built at DubHacks '26 with an SO-101 follower arm, a side camera and a wrist camera.

<p align="center">
  <img width="600" alt="Teleoperation demo" src="https://github.com/user-attachments/assets/088da4cc-9af4-4d11-8d19-646a16ce3f13" />
  <br />
  <sub>Collecting training demonstrations: an operator drives the leader arm (left) while the SO-101 follower (right) mirrors it.</sub>
</p>

<p align="center">
  <img width="600" alt="Battery Catcher setup" src="https://github.com/user-attachments/assets/a72616cf-2059-445d-a74c-34c1bf2fc8c7" />
  <br />
  <sub>Rerun showing a recorded teleoperation episode: front + wrist camera feeds and joint positions.</sub>
</p>

## What's here

| File | What it does |
|---|---|
| `run.py` | Runs a fine-tuned Perceptron Isaac 0.5 policy on the SO-101: predicts an action chunk from the cameras and joint state, plays it, repeats. Has a `--dry-run` mode and per-tick motion limits. |
| `upload.py` | Uploads a locally recorded LeRobot dataset to the Perceptron Isaac robotics service for fine-tuning. |
| `claude_bridge.py` | A file-based bridge that lets an AI agent drive the arm with safe, whitelisted commands (move, gripper, snapshot). It uses smooth interpolation, speed caps and joint-limit clamping. |
| `skills/so101-pick-place/` | Instructions and inverse-kinematics helpers for picking up AA batteries and dropping them in a box using direct control. |


## My role
- Recorded teleoperated demonstrations with LeRobot (side + wrist cameras) and fine-tuned Perceptron's Isaac 0.5 VLA on them
- Built the 30 Hz inference client (`run.py`) that streams both camera feeds and joint positions to the deployed policy and executes the returned 6-joint action chunks
- Added real-time chunking and joint-speed safety limits

  
## Setup

```bash
git clone https://github.com/Sparkaide/battery-catcher.git
cd battery-catcher
git clone https://github.com/huggingface/lerobot.git
pip install -e "./lerobot[feetech]"
export TEAM_API_KEY=...   # Perceptron team key; never commit it
python run.py --deployment dep_xxx --dry-run
```
