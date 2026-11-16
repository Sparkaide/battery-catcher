"""Send one command to the running claude_bridge.py and build a viewable image.
usage: do.py '<json command>' [--crop x0,y0,x1,y1] [--timeout 90]
Prints compact JSON (state/ok/error) and the image paths (relative to the trash-robot folder)
to stage + view: view/<tag>.jpg = side|wrist side by side (yellow ticks every 80 px);
with --crop also view/z.jpg = 2.5x zoom of that side-camera region | wrist."""
import json, sys, time, os, pathlib
HERE = pathlib.Path(__file__).resolve().parent
B = pathlib.Path(os.environ.get("BRIDGE_DIR", HERE.parents[2] / "bridge"))
args = sys.argv[1:]; crop = None; timeout = 90.0
if "--crop" in args: i = args.index("--crop"); crop = tuple(int(v) for v in args[i+1].split(",")); del args[i:i+2]
if "--timeout" in args: i = args.index("--timeout"); timeout = float(args[i+1]); del args[i:i+2]
cmd = json.loads(args[0])
cid = time.strftime("%Y%m%d%H%M%S") + f"{int(time.time()*1000)%1000:03d}"
tmp = B/"cmd"/f"{cid}.tmp"; tmp.write_text(json.dumps(cmd)); tmp.rename(B/"cmd"/f"{cid}.json")
r = B/"resp"/f"{cid}.json"; t0 = time.time()
while not r.exists():
    if time.time() - t0 > timeout: print(json.dumps({"ok": False, "error": "TIMEOUT - is claude_bridge.py running?"})); sys.exit(1)
    time.sleep(0.1)
o = json.loads(r.read_text())
for k in ("limits", "start_pose", "tb"): o.pop(k, None)
print(json.dumps(o))
sn = o.get("snaps")
if sn and "side" in sn and "wrist" in sn:
    from PIL import Image, ImageDraw
    S = B/"snaps"; tag = sn["side"].rsplit("_", 1)[0]; V = B/"view"; V.mkdir(exist_ok=True)
    a = Image.open(S/sn["side"]); b = Image.open(S/sn["wrist"])
    out = Image.new("RGB", (1280, 480)); out.paste(a, (0, 0)); out.paste(b, (640, 0)); d = ImageDraw.Draw(out)
    for x in range(0, 1280, 80): d.line([(x, 0), (x, 6)], fill=(255, 255, 0))
    for y in range(0, 480, 80): d.line([(0, y), (6, y)], fill=(255, 255, 0)); d.line([(640, y), (646, y)], fill=(255, 255, 0))
    out.save(V/f"{tag}.jpg", quality=85); print("VIEW bridge/view/%s.jpg" % tag)
    if crop:
        z = Image.new("RGB", (1200, 450)); z.paste(a.crop(crop).resize((600, 450)), (0, 0)); z.paste(b.resize((600, 450)), (600, 0))
        z.save(V/"z.jpg", quality=85); print("ZOOM bridge/view/z.jpg")
