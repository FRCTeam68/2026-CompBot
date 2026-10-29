#!/usr/bin/env python3
"""
Find rotation windows in each spot log, query TagPoses for each camera,
and report which tags are seen across the full rotation segment.

Key approach:
- TagPoses/length is the authoritative current tag count (logged whenever it
  changes).  Unlike slot coordinates, it goes to 0 when tags are lost.
- We use length during a "well-settled" sub-window (skipping the first 5s and
  last 2s of the settled segment) to determine how many slots are active.
- Slot coordinates use AKit hold-last-value (ClaudeScope returns prior values),
  so even a tag first logged just before seg_t0 is correctly picked up.
"""

import subprocess, json, math, os, bisect

CS = r"C:/Users/frc68/.claude/plugins/cache/thefrc-suite/thefrc-suite/df63d24d0d2a/ClaudeScope/ClaudeScope.exe"
FMAP_PATH = r"C:\FRC\2026\Code\2026-CompBot\other\LL\FRC2026_WELDED.fmap"
HALF_LEN = 16.541 / 2
HALF_WID = 8.014  / 2
MAX_MATCH_DIST = 0.40   # m

SPOTS = {
    "Spot1": r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot1.wpilog",
    "Spot2": r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot2.wpilog",
    "Spot3": r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot3.wpilog",
}
CAMERAS     = ["limelight-threeg", "limelight-four"]
CANON_ORDER = [180, 270, 0, 90]

SETTLE_SKIP_US = 5_000_000   # skip first 5s of segment (rotation approach)
SETTLE_TRIM_US = 2_000_000   # skip last 2s of segment (next rotation approach)

# ── ClaudeScope helpers ───────────────────────────────────────────────────────

def cs_load(path):
    env = {**os.environ, "MSYS_NO_PATHCONV": "1"}
    r = subprocess.run([CS, "load", path], capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise RuntimeError(f"load failed: {r.stderr}\n{r.stdout}")
    return json.loads(r.stdout)["session_id"]

def cs_range(sid, key, t0, t1):
    env = {**os.environ, "MSYS_NO_PATHCONV": "1"}
    r = subprocess.run([CS, "range", key, "--session", sid,
                        "--start", str(t0), "--end", str(t1)],
                       capture_output=True, text=True, env=env)
    if r.returncode != 0:
        return None
    v = json.loads(r.stdout).get(key)
    return v if v is not None else []

# ── Tag table ─────────────────────────────────────────────────────────────────

def load_tag_table():
    """
    Limelight TagPoses coordinate system:
      x = fmap_x + HALF_LEN  (same as WPILib x)
      y = fmap_y + HALF_WID  (NO y-flip — opposite to WPILib y = HALF_WID - fmap_y)
    """
    with open(FMAP_PATH) as f:
        fmap = json.load(f)
    tags = {}
    for fid in fmap.get("fiducials", []):
        t = fid["transform"]
        tags[fid["id"]] = {"x": t[3] + HALF_LEN, "y": t[7] + HALF_WID}
    return tags

TAG_TABLE = load_tag_table()

def nearest_tag(ox, oy):
    best_id, best_d = None, 999.0
    for tid, tp in TAG_TABLE.items():
        d = math.hypot(ox - tp["x"], oy - tp["y"])
        if d < best_d:
            best_d, best_id = d, tid
    return best_id, best_d

# ── Window detection ──────────────────────────────────────────────────────────

def angle_diff(a, b):
    return (a - b + 180) % 360 - 180

def find_rotation_windows(sid):
    """
    Pull full-log drive rotation; find longest stable heading segment for
    each of 180/270/0/90 deg.  Returns list of (canon, info_dict) in log order.
    """
    heading_key = "/RealOutputs/RobotPose/Drive/rotation/value"
    raw = cs_range(sid, heading_key, 0, 0)
    if not raw:
        print("  ERROR: heading key not found")
        return []

    pts = sorted(
        [(int(s["timestamp"]), math.degrees(s["value"]))
         for s in raw if isinstance(s.get("value"), (int, float))]
    )

    TOLERANCE = 20.0
    GAP_US    = 2_000_000
    WINDOW_US = 10_000_000

    by_canon = {}
    for canon in CANON_ORDER:
        near = [(t, d) for t, d in pts if abs(angle_diff(d, canon)) < TOLERANCE]
        if not near:
            print(f"  WARNING: no samples near {canon} deg")
            continue

        segments = []
        seg = [near[0]]
        for i in range(1, len(near)):
            if near[i][0] - near[i-1][0] > GAP_US:
                segments.append(seg)
                seg = [near[i]]
            else:
                seg.append(near[i])
        segments.append(seg)

        best_seg = max(segments, key=lambda s: s[-1][0] - s[0][0])
        seg_t0, seg_t1 = best_seg[0][0], best_seg[-1][0]

        # Ref window: last 10s of segment (rotation approach detections cleared)
        w1 = seg_t1
        w0 = max(seg_t0, w1 - WINDOW_US)

        median_hdg = sorted(d for _, d in best_seg)[len(best_seg) // 2]
        by_canon[canon] = {
            "ref_t0": w0, "ref_t1": w1,
            "seg_t0": seg_t0, "seg_t1": seg_t1,
            "median_hdg": median_hdg,
            "n_pts": len(best_seg),
        }

    return sorted(by_canon.items(), key=lambda kv: kv[1]["seg_t0"])

# ── Tag query ─────────────────────────────────────────────────────────────────

def get_tags_in_window(sid, cam, ts_cutoff, seg_t1):
    """
    Determine which tags are genuinely visible during the settled period.

    Strategy:
    1. Query TagPoses/length in the "well-settled" sub-window (ts_cutoff+5s to
       seg_t1-2s).  This scalar is reliable — it goes to 0 when no tags are
       detected, even when slot coordinates are stale.  The max length over
       this window is the number of simultaneously visible tags.
    2. For each slot 0..max_len-1, take the LAST x/y with timestamp >=
       ts_cutoff from the full settled period [ts_cutoff, seg_t1].  ClaudeScope
       returns prior values with their original timestamps, so tags first
       logged during the rotation approach (just before seg_t0) are included.
    """
    base = f"/RealOutputs/Vision/{cam}/TagPoses"

    well_t0 = ts_cutoff + SETTLE_SKIP_US
    well_t1 = max(well_t0 + 1_000_000, seg_t1 - SETTLE_TRIM_US)

    # Step 1: max active slot count during well-settled period
    len_raw = cs_range(sid, f"{base}/length", well_t0, well_t1)
    if len_raw is None:
        return []

    len_series = sorted(
        [(int(s["timestamp"]), int(float(s["value"]))) for s in (len_raw or [])
         if isinstance(s.get("value"), (int, float))],
        key=lambda p: p[0]
    )
    len_valid = [v for ts, v in len_series if ts >= ts_cutoff]
    max_len = max(len_valid, default=0)

    if max_len == 0:
        return []

    # Step 2: read positions for each active slot
    seen = set()
    for slot in range(max_len):
        sx_raw = cs_range(sid, f"{base}/{slot}/translation/x", ts_cutoff, seg_t1)
        sy_raw = cs_range(sid, f"{base}/{slot}/translation/y", ts_cutoff, seg_t1)
        if sx_raw is None or sy_raw is None:
            break
        if not sx_raw and not sy_raw:
            continue

        x_series = sorted(
            [(int(s["timestamp"]), float(s["value"])) for s in (sx_raw or [])
             if isinstance(s.get("value"), (int, float))],
            key=lambda p: p[0]
        )
        y_series = sorted(
            [(int(s["timestamp"]), float(s["value"])) for s in (sy_raw or [])
             if isinstance(s.get("value"), (int, float))],
            key=lambda p: p[0]
        )
        if not x_series or not y_series:
            continue

        # Last y value with timestamp >= ts_cutoff
        y_valid = [(yt, yv) for yt, yv in y_series if yt >= ts_cutoff]
        if not y_valid:
            continue
        yt, yv = y_valid[-1]

        x_ts = [p[0] for p in x_series]
        idx = max(0, bisect.bisect_right(x_ts, yt) - 1)
        ox = x_series[idx][1]

        tid, dist = nearest_tag(ox, yv)
        if dist < MAX_MATCH_DIST:
            seen.add(tid)

    return sorted(seen)

# ── Main ──────────────────────────────────────────────────────────────────────

def us_s(us):
    return f"{us/1e6:.2f}s"

def main():
    all_results = {}

    for spot in ["Spot1", "Spot2", "Spot3"]:
        print(f"\n{'='*70}")
        print(f"  {spot}  — loading {SPOTS[spot]}")
        print(f"{'='*70}")
        sid = cs_load(SPOTS[spot])
        print(f"  session: {sid[:8]}...")

        ordered = find_rotation_windows(sid)
        all_results[spot] = {}

        for i, (canon, w) in enumerate(ordered):
            prev_seg_t1 = ordered[i-1][1]["seg_t1"] if i > 0 else 0
            seg_t1 = w["seg_t1"]

            print(f"\n  {canon:>3}deg  segment {us_s(w['seg_t0'])}-{us_s(seg_t1)}"
                  f"  median={w['median_hdg']:.1f}deg  n={w['n_pts']}"
                  f"  cutoff={us_s(prev_seg_t1)}")
            print(f"        ref 10s: {us_s(w['ref_t0'])} - {us_s(w['ref_t1'])}"
                  f"  [{w['ref_t0']} - {w['ref_t1']} us]")

            row = {"t0": w["ref_t0"], "t1": w["ref_t1"],
                   "seg_t0": w["seg_t0"], "seg_t1": seg_t1}
            for cam in CAMERAS:
                tags = get_tags_in_window(sid, cam, prev_seg_t1, seg_t1)
                row[cam] = tags
                label = "LL3G" if "threeg" in cam else "LL4 "
                print(f"        {label}: {tags if tags else '(none)'}")

            all_results[spot][canon] = row

    print(f"\n\n{'='*90}")
    print("SUMMARY TABLE")
    print(f"{'='*90}")
    print(f"{'Yaw':>4}  {'Spot':<6}  {'10s ref window (us)':<36}  {'LL3G':^18}  {'LL4':^18}")
    print("-" * 90)
    for canon in CANON_ORDER:
        for spot in ["Spot1", "Spot2", "Spot3"]:
            res = all_results.get(spot, {}).get(canon)
            if not res:
                print(f"{canon:>4}  {spot:<6}  {'NOT FOUND':<36}  {'?':^18}  {'?':^18}")
                continue
            win = f"{res['t0']} - {res['t1']}"
            ll3g = str(res.get("limelight-threeg", []))
            ll4  = str(res.get("limelight-four",  []))
            print(f"{canon:>4}  {spot:<6}  {win:<36}  {ll3g:^18}  {ll4:^18}")

if __name__ == "__main__":
    main()
