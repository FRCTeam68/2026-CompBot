#!/usr/bin/env python3
"""
Identify which AprilTag IDs each camera sees at each rotation window,
by matching TagPoses field positions against the fmap tag positions.
"""

import subprocess, json, math, os, sys

CS = r"C:/Users/frc68/.claude/plugins/cache/thefrc-suite/thefrc-suite/df63d24d0d2a/ClaudeScope/ClaudeScope.exe"

def cs(*args):
    env = {**os.environ, "MSYS_NO_PATHCONV": "1"}
    r = subprocess.run([CS, *args], capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise RuntimeError(f"ClaudeScope '{args[0]}' failed:\n{r.stderr}\n{r.stdout}")
    return json.loads(r.stdout)

def load_log(path):
    return cs("load", path)["session_id"]

def cs_range(sid, key, t0=0, t1=0):
    env = {**os.environ, "MSYS_NO_PATHCONV": "1"}
    r = subprocess.run([CS, "range", key, "--session", sid,
                        "--start", str(t0), "--end", str(t1)],
                       capture_output=True, text=True, env=env)
    if r.returncode != 0:
        return None  # key not found or other error
    result = json.loads(r.stdout)
    v = result.get(key)
    return v if v is not None else []

def disconnect(sid):
    try: cs("disconnect", "--session", sid)
    except: pass

# ---- Build fmap tag position table (WPILib meters) ----------------------
# From FRC2026_WELDED.fmap. Transform is 4x4 row-major.
# Translation: t[3]=x, t[7]=y, t[11]=z
# Convention: WPILib_x = fmap_x + HALF_LEN,  WPILib_y = HALF_WID - fmap_y
HALF_LEN = 16.541 / 2   # 8.2705m
HALF_WID = 8.014  / 2   # 4.007m
FMAP_PATH = r"C:\FRC\2026\Code\2026-CompBot\other\LL\FRC2026_WELDED.fmap"

def load_tag_table(fmap_path):
    with open(fmap_path) as f:
        fmap = json.load(f)
    tags = {}
    for fid in fmap.get("fiducials", []):
        t = fid["transform"]
        fmap_x, fmap_y, fmap_z = t[3], t[7], t[11]
        wx = fmap_x + HALF_LEN
        wy = HALF_WID - fmap_y
        wz = fmap_z
        yaw = math.degrees(math.atan2(t[4], t[0]))
        tags[fid["id"]] = {"x": wx, "y": wy, "z": wz, "yaw": yaw}
    return tags

TAG_TABLE = load_tag_table(FMAP_PATH)

def nearest_tag(ox, oy):
    """Return (tag_id, dist) for the fmap tag closest to observed x,y."""
    best_id, best_d = None, 999
    for tid, tp in TAG_TABLE.items():
        d = math.hypot(ox - tp["x"], oy - tp["y"])
        if d < best_d:
            best_d = d
            best_id = tid
    return best_id, best_d

# ---- Known rotation windows (µs) ---------------------------------------
WINDOWS = {
    "Spot1": {
          0: (int(235.9e6), int(267.4e6)),
         90: (int(268.3e6), int(324.4e6)),
        180: (int(124.6e6), int(199.3e6)),
        -90: (int(200.2e6), int(231.5e6)),
    },
    "Spot2": {
          0: (int(1108.1e6), int(1140.3e6)),
         90: (int(1141.2e6), int(1197.9e6)),
        180: (int(1026.9e6), int(1056.2e6)),
        -90: (int(1059.3e6), int(1107.3e6)),
    },
    "Spot3": {
          0: (int(1825.4e6), int(1862.8e6)),
         90: (int(1863.7e6), int(1904.8e6)),
        180: (int(1755.9e6), int(1796.6e6)),
        -90: (int(1797.4e6), int(1824.4e6)),
    },
}

SPOTS = [
    ("Spot1", r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot1.wpilog"),
    ("Spot2", r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot2.wpilog"),
    ("Spot3", r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot3.wpilog"),
]

CAMERAS = ["limelight-threeg", "limelight-four"]
CANONICAL = [0, 90, 180, -90]
DEG_LABEL = {0: "  0°", 90: " 90°", 180: "180°", -90: "270°"}
MAX_MATCH_DIST = 0.30  # m — max distance to accept a fmap match

def get_tags_in_window(sid, cam, t0, t1):
    """
    Query TagPoses for a camera in a time window.
    Returns a sorted list of unique identified tag IDs (matched within MAX_MATCH_DIST).
    """
    base = f"/RealOutputs/Vision/{cam}/TagPoses"
    # Check how many tag slots are in the log
    tag_ids_seen = set()
    for slot in range(6):  # up to 6 simultaneous tags
        key_x = f"{base}/{slot}/translation/x"
        key_y = f"{base}/{slot}/translation/y"
        sx = cs_range(sid, key_x, t0, t1)
        sy = cs_range(sid, key_y, t0, t1)
        if sx is None or sy is None:
            break
        if not sx or not sy:
            break
        # Build lookup by timestamp
        y_by_ts = {s["timestamp"]: s["value"] for s in sy
                   if isinstance(s.get("value"), (int, float))}
        for s in sx:
            if not isinstance(s.get("value"), (int, float)):
                continue
            ox = s["value"]
            oy = y_by_ts.get(s["timestamp"])
            if oy is None:
                continue
            tid, dist = nearest_tag(ox, oy)
            if dist < MAX_MATCH_DIST:
                tag_ids_seen.add(tid)
    return sorted(tag_ids_seen)

def main():
    print("\nFMAP tag positions (WPILib m, red-side tags):")
    for tid in sorted(TAG_TABLE):
        if tid <= 16:
            tp = TAG_TABLE[tid]
            print(f"  Tag {tid:2d}: x={tp['x']:.3f} y={tp['y']:.3f} yaw={tp['yaw']:.0f}deg")

    print()
    results = {}  # {spot_name: {cam: {canon: [tag_ids]}}}

    for spot_name, log_path in SPOTS:
        print(f"\n{'='*60}")
        print(f"  {spot_name}")
        print(f"{'='*60}")
        sid = load_log(log_path)
        results[spot_name] = {}
        try:
            for cam in CAMERAS:
                results[spot_name][cam] = {}
                print(f"\n  {cam}:")
                for canon in CANONICAL:
                    t0, t1 = WINDOWS[spot_name][canon]
                    tags = get_tags_in_window(sid, cam, t0, t1)
                    results[spot_name][cam][canon] = tags
                    print(f"    {DEG_LABEL[canon]}: {tags if tags else '(none detected)'}")
        finally:
            disconnect(sid)

    # Summary table
    print(f"\n\n{'='*80}")
    print("TAG VISIBILITY SUMMARY")
    print(f"{'='*80}")
    for canon in CANONICAL:
        deg = canon if canon >= 0 else 360 + canon
        print(f"\n  WPILib {deg}deg rotation:")
        print(f"  {'Camera':<20} {'Spot1':<20} {'Spot2':<20} {'Spot3':<20}")
        for cam in CAMERAS:
            row = []
            for spot_name, _ in SPOTS:
                tags = results.get(spot_name, {}).get(cam, {}).get(canon, [])
                row.append(str(tags) if tags else "(none)")
            print(f"  {cam:<20} {row[0]:<20} {row[1]:<20} {row[2]:<20}")

if __name__ == "__main__":
    main()
