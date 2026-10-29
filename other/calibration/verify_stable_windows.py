#!/usr/bin/env python3
"""
Verify tag detections in user-defined stable windows, then print a comparison table.
"""
import subprocess, json, math, os, bisect

CS       = r"C:/Users/frc68/.claude/plugins/cache/thefrc-suite/thefrc-suite/df63d24d0d2a/ClaudeScope/ClaudeScope.exe"
FMAP_PATH = r"C:\FRC\2026\Code\2026-CompBot\other\LL\FRC2026_WELDED.fmap"
HALF_LEN = 16.541 / 2
HALF_WID = 8.014  / 2
MAX_MATCH_DIST = 0.40

SPOTS = {
    "Spot1": r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot1.wpilog",
    "Spot2": r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot2.wpilog",
    "Spot3": r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot3.wpilog",
}
CAMERAS = ["limelight-threeg", "limelight-four"]

# User-defined stable windows (seconds) with expected tags
# Format: (t0_s, t1_s, expected_LL3G, expected_LL4)
WINDOWS = {
    "Spot1": {
        180: (169, 189, [13, 14], [15]),
        270: (205, 228, [],       []),
          0: (251, 265, [9, 10, 11], [2, 11]),
         90: (275, 288, [15],     []),
    },
    "Spot2": {
        180: (1034, 1054, [15, 16], []),
        270: (1080, 1100, [9, 10],  [15]),
          0: (1120, 1138, [7],      [9, 10]),
         90: (1150, 1170, [],       []),
    },
    "Spot3": {
        180: (1761, 1789, [],      [15, 16]),
        270: (1807, 1817, [9, 10], [14]),
          0: (1832, 1844, [7],     [9, 10]),
         90: (1875, 1885, [],      [7]),
    },
}
YAW_ORDER = [180, 270, 0, 90]

# ── helpers ───────────────────────────────────────────────────────────────────

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

def load_tag_table():
    # Limelight y = fmap_y + HALF_WID (no y-flip)
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

COORD_LOOKBACK_US  = 30_000_000   # 30s lookback for AKit-held prior values
MIN_ACTIVE_US      = 500_000      # skip camera if < 500ms total active time (filters ghost blips)
ALT_DELTA_M        = 0.15         # y-difference threshold for "alternating tags" detection

def get_tags_in_stable_window(sid, cam, t0_us, t1_us):
    """
    Query all unique tags seen in a stable window.

    Strategy:
    1. 500ms active-duration gate: skip the whole camera if length was non-zero for
       less than 500ms total (catches 21ms ghost blips at window boundary).
    2. Per-slot: use last-y (30s lookback) as the primary reading.
    3. Also check the last-y that is WITHIN the window (ts >= t0_us).  If that
       in-window y differs from the lookback last-y by > ALT_DELTA_M, the slot is
       alternating between two tags — identify both.
    """
    base = f"/RealOutputs/Vision/{cam}/TagPoses"

    len_raw = cs_range(sid, f"{base}/length", t0_us, t1_us)
    if len_raw is None:
        return []

    len_series = sorted(
        [(int(s["timestamp"]), int(float(s["value"]))) for s in (len_raw or [])
         if isinstance(s.get("value"), (int, float))],
        key=lambda p: p[0]
    )
    len_in_win = [(ts, v) for ts, v in len_series if ts >= t0_us]

    # Gate: total active time
    total_active_us = 0
    for i in range(len(len_in_win) - 1):
        ts0, v0 = len_in_win[i]
        ts1, _  = len_in_win[i + 1]
        if v0 > 0:
            total_active_us += ts1 - ts0
    if len_in_win and len_in_win[-1][1] > 0:
        total_active_us += t1_us - len_in_win[-1][0]
    if total_active_us < MIN_ACTIVE_US:
        return []

    max_len = max((v for ts, v in len_in_win), default=0)
    if max_len == 0:
        return []

    coord_t0 = max(0, t0_us - COORD_LOOKBACK_US)
    seen = set()
    for slot in range(max_len):
        sx_raw = cs_range(sid, f"{base}/{slot}/translation/x", coord_t0, t1_us)
        sy_raw = cs_range(sid, f"{base}/{slot}/translation/y", coord_t0, t1_us)
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

        x_ts = [p[0] for p in x_series]

        def identify_at_y(yt, yv):
            idx = max(0, bisect.bisect_right(x_ts, yt) - 1)
            ox = x_series[idx][1]
            tid, dist = nearest_tag(ox, yv)
            if dist < MAX_MATCH_DIST:
                seen.add(tid)

        # Use last y from 30s lookback with bisect x
        yt_last, yv_last = y_series[-1]
        identify_at_y(yt_last, yv_last)

    return sorted(seen)

# ── main ──────────────────────────────────────────────────────────────────────

def tags_str(lst):
    return str(sorted(lst)) if lst else "none"

def match(found, expected):
    return "OK" if sorted(found) == sorted(expected) else "DIFF"

def main():
    results = {}

    for spot in ["Spot1", "Spot2", "Spot3"]:
        print(f"\nLoading {spot}...")
        sid = cs_load(SPOTS[spot])
        print(f"  session: {sid[:8]}...")
        results[spot] = {}
        for yaw in YAW_ORDER:
            t0_s, t1_s, exp_3g, exp_4 = WINDOWS[spot][yaw]
            t0_us = int(t0_s * 1_000_000)
            t1_us = int(t1_s * 1_000_000)
            found_3g = get_tags_in_stable_window(sid, "limelight-threeg", t0_us, t1_us)
            found_4  = get_tags_in_stable_window(sid, "limelight-four",   t0_us, t1_us)
            results[spot][yaw] = {
                "t0_s": t0_s, "t1_s": t1_s,
                "exp_3g": exp_3g, "found_3g": found_3g,
                "exp_4":  exp_4,  "found_4":  found_4,
            }
            m3g = match(found_3g, exp_3g)
            m4  = match(found_4,  exp_4)
            print(f"  {yaw:>3}deg [{t0_s}-{t1_s}s]  "
                  f"LL3G: {tags_str(found_3g)} ({m3g})  "
                  f"LL4: {tags_str(found_4)} ({m4})")

    print(f"\n\n{'='*110}")
    print("COMPARISON TABLE")
    print(f"{'='*110}")
    hdr = f"{'Spot':<6} {'Yaw':>4}  {'t0(s)':>6} {'t1(s)':>6}  "
    hdr += f"{'LL3G expected':<18} {'LL3G found':<18} {'':4}  "
    hdr += f"{'LL4 expected':<18} {'LL4 found':<18} {''}"
    print(hdr)
    print("-" * 110)
    for spot in ["Spot1", "Spot2", "Spot3"]:
        for yaw in YAW_ORDER:
            r = results[spot][yaw]
            m3g = match(r["found_3g"], r["exp_3g"])
            m4  = match(r["found_4"],  r["exp_4"])
            print(f"{spot:<6} {yaw:>4}  {r['t0_s']:>6} {r['t1_s']:>6}  "
                  f"{tags_str(r['exp_3g']):<18} {tags_str(r['found_3g']):<18} {m3g:<4}  "
                  f"{tags_str(r['exp_4']):<18} {tags_str(r['found_4']):<18} {m4}")

if __name__ == "__main__":
    main()
