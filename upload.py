"""Direct-upload a local LeRobot dataset to the Perceptron Isaac robotics service.

Usage:
  export TEAM_API_KEY=...   # your Perceptron team key
  python upload_dataset.py ~/.cache/huggingface/lerobot/local/battery_catcher_all battery-all

Follows robotics.perceptron.inc/docs and the Isaac 0.5 workshop slides:
  1. POST /v1/datasets {source_kind: upload, name} with an Idempotency-Key  -> dataset id
  2. POST /v1/datasets/{id}/uploads {files:[{path,size_bytes,sha256}]}       -> .files[] with url, method, headers
  3. PUT each file to its signed URL with exactly the returned headers, NO team key.
     A 303 is an opaque poll URL: follow its Location with no credentials, never resend the body.
  4. POST /v1/datasets/{id}/finalize  (only after every PUT finished; never upload to it again)
  5. Poll GET /v1/datasets/{id} every few seconds until ready/failed, then GET .../validation
"""
import hashlib
import json
import os
import sys
import time
from pathlib import Path
from urllib.parse import urljoin

import requests

CONTROL_URL = "https://robotics.perceptron.inc"
BASE = CONTROL_URL + "/v1"
ALLOWED = {".json", ".jsonl", ".parquet", ".mp4", ".png", ".jpg", ".jpeg"}
BATCH = 100  # under the 500-per-reservation limit; signed URLs last 900 s, so upload each batch right away

KEY = os.environ.get("TEAM_API_KEY")
if not KEY:
    sys.exit("Set TEAM_API_KEY first: export TEAM_API_KEY=...")
AUTH = {"Authorization": f"Bearer {KEY}"}


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def follow_303(resp: requests.Response, origin: str) -> requests.Response:
    """Poll an opaque 303 Location exactly as given, with no team key and no body, until it completes."""
    while resp.status_code == 303:
        loc = urljoin(origin, resp.headers["Location"])
        time.sleep(2)
        resp = requests.get(loc, allow_redirects=False, timeout=300)
        origin = loc
    return resp


def control(method: str, path: str, **kw) -> requests.Response:
    url = BASE + path
    r = requests.request(method, url, headers={**AUTH, **kw.pop("headers", {})},
                         allow_redirects=False, timeout=300, **kw)
    r = follow_303(r, url)
    if r.status_code >= 400:
        sys.exit(f"{method} {path} failed {r.status_code}: {r.text[:2000]}")
    return r


def main(root: str, name: str) -> None:
    root_p = Path(root).expanduser().resolve()
    if not (root_p / "meta" / "info.json").is_file():
        sys.exit(f"{root_p} has no meta/info.json; point at the dataset folder itself.")

    # Regular files only (no symlinks), allowed types, nonzero size; paths relative to the export root.
    files, skipped = [], []
    for p in sorted(root_p.rglob("*")):
        if p.is_symlink() or not p.is_file():
            continue
        if p.suffix.lower() not in ALLOWED or p.stat().st_size == 0:
            skipped.append(str(p.relative_to(root_p)))
            continue
        files.append(p)
    total_gb = sum(p.stat().st_size for p in files) / 1e9
    print(f"{len(files)} files ({total_gb:.2f} GB) to upload; skipped {len(skipped)}: {skipped[:10]}")

    # 1. Register. Same Idempotency-Key + same body returns the same dataset, so a rerun after a crash is safe.
    r = control("POST", "/datasets", json={"name": name, "source_kind": "upload"},
                headers={"Idempotency-Key": f"upload-{name}"})
    ds = r.json()
    ds_id = ds.get("id") or ds.get("dataset_id")
    state = ds.get("state") or ds.get("status")
    print("Dataset id:", ds_id, "| state:", state, "(save the id)")
    if state in ("queued", "ingesting", "ready", "failed"):
        sys.exit("This dataset is already past uploading (finalized). Use a new name for a new upload.")

    # 2 + 3. Reserve a batch, then PUT each file to its signed URL right away.
    for i in range(0, len(files), BATCH):
        batch = files[i:i + BATCH]
        spec = [{"path": p.relative_to(root_p).as_posix(), "size_bytes": p.stat().st_size, "sha256": sha256(p)}
                for p in batch]
        res = control("POST", f"/datasets/{ds_id}/uploads", json={"files": spec}).json()
        slots = res.get("files")
        if not isinstance(slots, list):
            sys.exit("Unexpected reservation response:\n" + json.dumps(res, indent=2)[:2000])
        by_path = {s["path"]: s for s in slots}
        for p in batch:
            rel = p.relative_to(root_p).as_posix()
            s = by_path.get(rel)
            if s is None:
                sys.exit(f"No signed URL returned for {rel}")
            url = urljoin(CONTROL_URL, s["url"])
            with open(p, "rb") as f:  # exact returned headers (incl. Content-Length); never the team key
                put = requests.request(s.get("method", "PUT"), url, headers=s.get("headers", {}),
                                       data=f, allow_redirects=False, timeout=1800)
            put = follow_303(put, url)
            if put.status_code >= 300:
                sys.exit(f"Upload of {rel} failed {put.status_code}: {put.text[:500]}\n"
                         "While the dataset is still 'uploading' you can rerun this script with the same name.")
            print(f"  uploaded {rel} ({p.stat().st_size / 1e6:.0f} MB)")

    # 4. Finalize only after every upload has completed.
    control("POST", f"/datasets/{ds_id}/finalize")
    print("Finalized. Ingest: validate, convert, split ~90/10 into train/validation...")

    # 5. Poll every few seconds; train only on 'ready'.
    while True:
        st = control("GET", f"/datasets/{ds_id}").json()
        state = st.get("state") or st.get("status")
        print("  state:", state)
        if state in ("ready", "failed"):
            break
        time.sleep(10)

    val = control("GET", f"/datasets/{ds_id}/validation").json()
    print("Validation:\n", json.dumps(val, indent=2)[:4000])
    print(f"\nDATASET_ID={ds_id}  (state: {state})")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
