#!/usr/bin/env python3
"""
Limelight camera-to-robot extrinsic calibration (rsf / rss / rsu).
Multi-rotation version: each log file contains 4 rotation segments at
WPILib headings ~0°, 90°, 180°, 270°. This script finds each stable
window, then computes rsf/rss independently per window.

Usage:
    python calibrate_cameras.py <spot1.wpilog> <spot2.wpilog> <spot3.wpilog>
"""

import subprocess, json, math, os, sys
import numpy as np

# --- ClaudeScope helper ---------------------------------------------------
CS = r"C:/Users/frc68/.claude/plugins/cache/thefrc-suite/thefrc-suite/df63d24d0d2a/ClaudeScope/ClaudeScope.exe"

def cs(*args):
    env = {**os.environ, "MSYS_NO_PATHCONV": "1"}
    r = subprocess.run([CS, *args], capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise RuntimeError(f"ClaudeScope '{args[0]}' failed:\n{r.stderr}\n{r.stdout}")
    return json.loads(r.stdout)

def load_log(path):
    return cs("load", path)["session_id"]

def cs_range(sid, key, start_us=0, end_us=0):
    result = cs("range", key, "--session", sid,
                "--start", str(start_us), "--end", str(end_us))
    return result.get(key, [])

def disconnect(sid):
    try: cs("disconnect", "--session", sid)
    except Exception: pass

# --- Field / robot geometry -----------------------------------------------
RED_WALL_X  = 16.541   # m: red scoring wall x
FIELD_WIDTH = 8.014    # m: inside face of high-y perimeter wall

def in2m(x): return x * 0.0254

# Known robot center positions.
# x_in / y_in = distance in inches FROM red scoring wall / FROM high-y wall.
SPOTS = [
    {"name": "Spot1", "x_in": 79,     "y_in": 101.5},
    {"name": "Spot2", "x_in": 61.75,  "y_in": 215},
    {"name": "Spot3", "x_in": 100.75, "y_in": 187.75},
]

def robot_wpilib(spot):
    """Robot center in WPILib field coordinates (m)."""
    x = RED_WALL_X - in2m(spot["x_in"])
    y = FIELD_WIDTH - in2m(spot["y_in"])
    return x, y

# Current VPR values on each camera (used to reconstruct camera field pos)
CURRENT_VPR = {
    "limelight-threeg": {"rsf": -0.27,      "rss":  0.27,      "rsu": 0.22},
    "limelight-four":   {"rsf": -0.291079,  "rss": -0.273426,  "rsu": 0.31586},
}

HEADING_KEY = "/RealOutputs/RobotPose/Drive/rotation/value"

# WPILib canonical headings (degrees, wrapped to [-180, 180])
#   0°  = intake faces toward red scoring wall (tags 13-16)
#  90°  = intake faces toward high-y wall (left wall from red)
# 180°  = intake faces toward blue driver station
# -90° / 270° = intake faces toward low-y wall (right wall from red)
CANONICAL_DEG = [0, 90, 180, -90]   # -90 is how WPILib logs 270°

ROT_LABEL = {
    0:   "  0deg (intake->red/tags13-16)",
    90:  " 90deg (intake->high-y/left from red)",
    180: "180deg (intake->blue)",
    -90: "270deg (intake->low-y/right from red)",
}

# --- Heading helpers -------------------------------------------------------
def extract_heading_rad(v):
    if isinstance(v, dict):
        for k in ("value", "radians", "rads"):
            if k in v: return float(v[k])
        rot = v.get("rotation")
        if isinstance(rot, dict):
            for k in ("value", "radians", "rads"):
                if k in rot: return float(rot[k])
    elif isinstance(v, (int, float)):
        return float(v)
    return None

def wrap180(d):
    """Wrap degrees to [-180, 180)."""
    d = d % 360
    return d - 360 if d >= 180 else d

def angular_diff(a, b):
    """Smallest unsigned angular difference in degrees."""
    return abs(wrap180(a - b))

# --- Rotation window detection --------------------------------------------
def find_rotation_windows(samples, tolerance=20.0, min_samples=15):
    """
    Scan heading samples and find the longest stable window near each
    canonical heading.

    Returns:
        dict { canonical_deg: (t_start_us, t_end_us, median_hdg_deg, n) }
    """
    # Convert to (timestamp_us, heading_deg_wrapped)
    points = []
    for s in samples:
        rad = extract_heading_rad(s["value"])
        if rad is not None:
            points.append((s["timestamp"], wrap180(math.degrees(rad))))
    if not points:
        return {}
    points.sort()

    windows = {}
    for canon in CANONICAL_DEG:
        best = None
        in_win = False
        w_start = w_end = None
        w_hdgs = []

        for ts, d in points:
            close = angular_diff(d, canon) <= tolerance
            if close:
                if not in_win:
                    in_win = True
                    w_start = ts
                    w_hdgs = []
                w_end = ts
                w_hdgs.append(d)
            else:
                if in_win:
                    if len(w_hdgs) >= min_samples:
                        if best is None or len(w_hdgs) > len(best[2]):
                            best = (w_start, w_end, w_hdgs[:])
                    in_win = False

        if in_win and len(w_hdgs) >= min_samples:
            if best is None or len(w_hdgs) > len(best[2]):
                best = (w_start, w_end, w_hdgs[:])

        if best:
            t0, t1, hdgs = best
            # Careful median for angles near ±180 boundary
            med_hdg = float(np.median(hdgs))
            windows[canon] = (t0, t1, med_hdg, len(hdgs))

    return windows

# --- MT2 pose extraction --------------------------------------------------
def get_median_scalar(samples, t_start, t_end):
    vals = [float(s["value"]) for s in samples
            if t_start <= s["timestamp"] <= t_end
            and isinstance(s["value"], (int, float))]
    if not vals:
        return None, 0
    return float(np.median(vals)), len(vals)

# --- Calibration math -----------------------------------------------------
def reconstruct_camera_pos(x_bot, y_bot, theta, rsf_c, rss_c):
    """Undo current VPR to get camera field XY from MT2 bot pose."""
    x_cam = x_bot + rsf_c * math.cos(theta) + rss_c * math.sin(theta)
    y_cam = y_bot + rsf_c * math.sin(theta) - rss_c * math.cos(theta)
    return x_cam, y_cam

def solve_offsets(x_cam, y_cam, x_R, y_R, theta):
    """Solve for rsf / rss given camera field pos, known robot pos, heading."""
    dx, dy = x_cam - x_R, y_cam - y_R
    rsf = dx * math.cos(theta) + dy * math.sin(theta)
    rss = dx * math.sin(theta) - dy * math.cos(theta)
    return rsf, rss

# --- Plausibility check ---------------------------------------------------
# Physical bounds based on known camera mount positions on the robot.
PLAUS_BOUNDS = {
    "limelight-threeg": {"rsf": (-0.40, -0.10), "rss": (0.10, 0.45)},
    "limelight-four":   {"rsf": (-0.55, -0.15), "rss": (-0.45, -0.10)},
}

def plausibility(cam_name, rsf, rss):
    b = PLAUS_BOUNDS[cam_name]
    rsf_ok = b["rsf"][0] < rsf < b["rsf"][1]
    rss_ok = b["rss"][0] < rss < b["rss"][1]
    if rsf_ok and rss_ok:   return "GOOD"
    if rsf_ok or rss_ok:    return "MARGINAL"
    return "IMPLAUSIBLE"

# --- Per-spot analysis ----------------------------------------------------
def analyze_spot(spot, log_path):
    """
    Returns:
        (spot_results, x_R, y_R)
        spot_results: { cam_name: { canonical_deg: result_dict } }
    """
    x_R, y_R = robot_wpilib(spot)
    print(f"\nLoading {spot['name']}: {log_path}")
    sid = load_log(log_path)
    spot_results = {}

    try:
        # ---- Detect rotation windows from heading data -------------------
        all_hdg = cs_range(sid, HEADING_KEY)
        if not all_hdg:
            print(f"  ERROR: no heading data at {HEADING_KEY}")
            return {}, x_R, y_R

        windows = find_rotation_windows(all_hdg)
        print(f"  Heading samples: {len(all_hdg)}")
        for c in CANONICAL_DEG:
            if c in windows:
                t0, t1, mh, n = windows[c]
                print(f"    {ROT_LABEL[c]}: n={n}, t={t0/1e6:.1f}–{t1/1e6:.1f}s, "
                      f"median={mh:.1f}°")
            else:
                print(f"    {ROT_LABEL[c]}: NOT FOUND")

        # ---- Per-camera calibration per rotation -------------------------
        for cam_name, vpr in CURRENT_VPR.items():
            cam_res = {}
            pk_x = f"/RealOutputs/Vision/{cam_name}/MegaTag2/RobotPose/0/translation/x"
            pk_y = f"/RealOutputs/Vision/{cam_name}/MegaTag2/RobotPose/0/translation/y"
            all_x = cs_range(sid, pk_x)
            all_y = cs_range(sid, pk_y)

            for canon in CANONICAL_DEG:
                if canon not in windows:
                    cam_res[canon] = {"status": "no_heading_window"}
                    continue

                t0, t1, mh, n_hdg = windows[canon]
                theta = math.radians(mh)

                x_bot, n_x = get_median_scalar(all_x, t0, t1)
                y_bot, n_y = get_median_scalar(all_y, t0, t1)

                if x_bot is None or n_x < 5:
                    cam_res[canon] = {"status": "no_mt2_data", "n_hdg": n_hdg, "n_mt2": n_x}
                    continue

                x_cam, y_cam = reconstruct_camera_pos(x_bot, y_bot, theta, vpr["rsf"], vpr["rss"])
                new_rsf, new_rss = solve_offsets(x_cam, y_cam, x_R, y_R, theta)
                plaus = plausibility(cam_name, new_rsf, new_rss)

                # MT2 bot pose residual (how far MT2 thinks robot is from known position)
                residual = math.hypot(x_bot - x_R, y_bot - y_R)

                cam_res[canon] = {
                    "status": "ok",
                    "rsf": new_rsf, "rss": new_rss, "rsu": vpr["rsu"],
                    "theta_deg": mh,
                    "x_bot": x_bot, "y_bot": y_bot,
                    "residual_m": residual,
                    "n_mt2": n_x,
                    "plausibility": plaus,
                }

            spot_results[cam_name] = cam_res

    finally:
        disconnect(sid)

    return spot_results, x_R, y_R

# --- Summary table --------------------------------------------------------
def print_summary_table(all_spot_results):
    print(f"\n{'='*80}")
    print("SUMMARY TABLE — rsf / rss per spot per rotation")
    print(f"{'='*80}")
    print()
    print("Rotation label convention (WPILib heading):")
    print("   0° = intake faces toward red driver station (tags 13-16)")
    print("  90° = intake faces toward high-y wall (left wall from red driver station)")
    print(" 180° = intake faces toward blue driver station")
    print(" 270° = intake faces toward low-y wall (right wall from red driver station)")
    print()

    col_w = 32

    for cam_name in CURRENT_VPR:
        vpr = CURRENT_VPR[cam_name]
        print(f"\n{'─'*80}")
        print(f"  {cam_name.upper()}")
        print(f"  Current VPR: rsf={vpr['rsf']:+.4f}  rss={vpr['rss']:+.4f}  rsu={vpr['rsu']:.4f}")
        print(f"{'─'*80}")

        header = f"  {'Rotation':<12}  {'Spot1':<{col_w}}  {'Spot2':<{col_w}}  {'Spot3':<{col_w}}"
        print(header)

        # Coord row
        coord_vals = []
        for spot, (sr, x_R, y_R) in all_spot_results:
            x_u = spot["x_in"]
            y_u = spot["y_in"]
            coord_vals.append(f"x={x_R:.3f}m y={y_R:.3f}m")
        print(f"  {'WPILib (m)':<12}  {coord_vals[0]:<{col_w}}  {coord_vals[1]:<{col_w}}  {coord_vals[2]:<{col_w}}")
        coord_vals2 = []
        for spot, _ in all_spot_results:
            coord_vals2.append(f"({spot['x_in']:.2f}\",{spot['y_in']:.2f}\") red-rt")
        print(f"  {'User (in)':<12}  {coord_vals2[0]:<{col_w}}  {coord_vals2[1]:<{col_w}}  {coord_vals2[2]:<{col_w}}")
        print()

        for canon in CANONICAL_DEG:
            deg_label = canon if canon >= 0 else 360 + canon
            row_cells = []
            for spot, (spot_results, x_R, y_R) in all_spot_results:
                cam_res = spot_results.get(cam_name, {})
                r = cam_res.get(canon)
                if r is None:
                    row_cells.append("—")
                elif r["status"] == "no_heading_window":
                    row_cells.append("no heading window")
                elif r["status"] == "no_mt2_data":
                    row_cells.append(f"no MT2 (n={r['n_mt2']})")
                else:
                    plaus = r["plausibility"]
                    cell = (f"rsf={r['rsf']:+.4f} rss={r['rss']:+.4f}\n"
                            f"                  "
                            f"  n={r['n_mt2']} res={r['residual_m']*100:.1f}cm [{plaus}]")
                    row_cells.append(cell)
            rot_str = f"{deg_label:3d}°"
            print(f"  {rot_str:<12}  {row_cells[0]:<{col_w}}  {row_cells[1]:<{col_w}}  {row_cells[2]:<{col_w}}")
            print()

    # Per-camera averages using only GOOD results
    print(f"\n{'='*80}")
    print("AVERAGES (GOOD results only, using all spots and all rotations)")
    print(f"{'='*80}")

    for cam_name in CURRENT_VPR:
        vpr = CURRENT_VPR[cam_name]
        rsf_vals, rss_vals = [], []
        used = []
        for spot, (spot_results, x_R, y_R) in all_spot_results:
            cam_res = spot_results.get(cam_name, {})
            for canon in CANONICAL_DEG:
                r = cam_res.get(canon)
                if r and r["status"] == "ok" and r["plausibility"] == "GOOD":
                    rsf_vals.append(r["rsf"])
                    rss_vals.append(r["rss"])
                    deg_label = canon if canon >= 0 else 360 + canon
                    used.append(f"{spot['name']}@{deg_label}°")

        if not rsf_vals:
            print(f"\n  {cam_name}: no GOOD results to average")
            continue

        rsf_mean = np.mean(rsf_vals)
        rss_mean = np.mean(rss_vals)
        rsf_std  = np.std(rsf_vals)
        rss_std  = np.std(rss_vals)

        print(f"\n  {cam_name}  (n={len(rsf_vals)} windows: {', '.join(used)})")
        print(f"    rsf:  {vpr['rsf']:+.4f}  →  {rsf_mean:+.4f}   Δ={rsf_mean-vpr['rsf']:+.4f}   std={rsf_std:.4f}")
        print(f"    rss:  {vpr['rss']:+.4f}  →  {rss_mean:+.4f}   Δ={rss_mean-vpr['rss']:+.4f}   std={rss_std:.4f}")
        print(f"    rsu:  {vpr['rsu']:+.4f}  (unchanged)")
        print(f"    Per-result rsf / rss:")
        for i, (rsf, rss) in enumerate(zip(rsf_vals, rss_vals)):
            print(f"      {used[i]}: rsf={rsf:+.4f}  rss={rss:+.4f}")

# --- Main -----------------------------------------------------------------
def main():
    if len(sys.argv) < 4:
        sys.exit("Usage: python calibrate_cameras.py <spot1.wpilog> <spot2.wpilog> <spot3.wpilog>")

    log_paths = sys.argv[1:4]
    all_spot_results = []

    for spot, path in zip(SPOTS, log_paths):
        result = analyze_spot(spot, path)
        all_spot_results.append((spot, result))

    print_summary_table(all_spot_results)

if __name__ == "__main__":
    main()
