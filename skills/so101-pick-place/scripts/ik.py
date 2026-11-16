"""Approximate planar IK/FK for the SO-101 (lerobot use_degrees=True, this arm's calibration).
Angles: shoulder_lift t1 (0 = upper arm vertical, + leans forward), elbow e (+ folds),
wrist_flex w (+ tilts gripper down/back).  Absolute link angles from horizontal-forward:
  upper arm a1 = 90 - t1 ; forearm a2 = -t1 - e ; gripper a3 = -t1 - e - w  (a3=-90 -> pointing straight down)
x = horizontal reach of the WRIST-FLEX axis from the shoulder-lift axis, z = its height above that axis (metres).
Verified empirically on this arm; good to ~1 cm, close the loop with the cameras."""
import math, json, sys
L1, L2 = 0.116, 0.135
def fk(t1, e, w):
    a1 = math.radians(90 - t1); a2 = math.radians(-t1 - e)
    return L1*math.cos(a1) + L2*math.cos(a2), L1*math.sin(a1) + L2*math.sin(a2), -t1 - e - w
def ik(x, z, a3=-90.0):
    d2 = x*x + z*z; d = math.sqrt(d2)
    if d > L1 + L2 - 0.005: raise ValueError("out of reach")
    g = math.acos(max(-1, min(1, (L1*L1 + L2*L2 - d2) / (2*L1*L2))))
    b = math.acos(max(-1, min(1, (L1*L1 + d2 - L2*L2) / (2*L1*d))))
    a1 = math.atan2(z, x) + b; a2 = a1 - (math.pi - g)          # elbow-up
    t1 = 90 - math.degrees(a1); e = -t1 - math.degrees(a2); w = -t1 - e - a3
    return t1, e, w
if __name__ == "__main__":
    if sys.argv[1] == "fk": print(fk(*map(float, sys.argv[2:5])))
    else:
        t1, e, w = ik(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]) if len(sys.argv) > 4 else -90)
        print(json.dumps({"shoulder_lift": round(t1,2), "elbow_flex": round(e,2), "wrist_flex": round(w,2)}))
