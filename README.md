# Battery Catcher

Battery-sorting robot arm built at DubHacks '26 with an SO-101 follower arm, a side camera and a wrist camera. Code for the Battery Catcher project.

## What's here

| File | What it does |
|---|---|
| `run.py` | Runs a fine-tuned Perceptron Isaac 0.5 policy on the SO-101: predicts an action chunk from the cameras and joint state, plays it, repeats. Has a `--dry-run` mode and per-tick motion limits. |
| `upload.py` | Uploads a locally recorded LeRobot dataset to the Perceptron Isaac robotics service for fine-tuning. |
| `claude_bridge.py` | A file-based bridge that lets an AI agent drive the arm with safe, whitelisted commands (move, gripper, snapshot). It uses smooth interpolation, speed caps and joint-limit clamping. |
| `skills/so101-pick-place/` | Instructions and inverse-kinematics helpers for picking up AA batteries and dropping them in a box using direct control. |

## Setup

```bash
git clone https://github.com/huggingface/lerobot.git
cd lerobot && pip install -e ".[feetech]"
export TEAM_API_KEY=...   # Perceptron team key; never commit it
python run.py --deployment dep_xxx --dry-run
```
