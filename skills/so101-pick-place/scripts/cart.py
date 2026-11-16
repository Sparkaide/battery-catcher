"""Print a bridge 'move' command for a Cartesian wrist target.
usage: cart.py X Z [PAN|-] [EXTRA_JSON] [--a3 -90] [--dur 2] [--nosnap]
Applies -3.5 deg elbow gravity compensation (the elbow otherwise settles ~4 deg more folded)."""
import sys, json, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ik import ik
args = sys.argv[1:]; a3 = -90.0; dur = 2.0; snap = True
if "--a3" in args: i = args.index("--a3"); a3 = float(args[i+1]); del args[i:i+2]
if "--dur" in args: i = args.index("--dur"); dur = float(args[i+1]); del args[i:i+2]
if "--nosnap" in args: args.remove("--nosnap"); snap = False
x, z = float(args[0]), float(args[1])
t1, e, w = ik(x, z, a3)
tgt = {"shoulder_lift": round(t1,2), "elbow_flex": round(e - 3.5,2), "wrist_flex": round(w,2)}
if len(args) > 2 and args[2] != "-": tgt["shoulder_pan"] = float(args[2])
if len(args) > 3: tgt.update(json.loads(args[3]))
print(json.dumps({"op": "move", "target": tgt, "duration": dur, "snap": snap}))
