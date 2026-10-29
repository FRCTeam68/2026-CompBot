#!/usr/bin/env python3
"""
Compute rsf/rss camera offset corrections for Limelight cameras.

For each stable calibration window where tags are expected:
  1. Query MegaTag2 botpose mean x,y during the window
     (fall back to MegaTag1 if MT2 has no data)
  2. Compare to known ground-truth robot center position
  3. Solve via least squares (3 unknowns: drsf, drss, Δrsyaw):
       err_x = drsf·cos(θ) − drss·sin(θ) + Δrsyaw·Jψ_x
       err_y = drsf·sin(θ) + drss·cos(θ) + Δrsyaw·Jψ_y
     where Jψ = mean(-d_i·sin(α_i), d_i·cos(α_i)) over expected tags,
     d_i = camera-to-tag distance, α_i = field-frame angle from camera to tag
  4. Output: recommended rsf   = current_rsf   + drsf
             recommended rss   = current_rss   + drss
             recommended rsyaw = current_rsyaw + Δrsyaw

Coordinate conventions:
  - θ  = robot field heading in radians (WPILib: CCW from field +x)
  - rsf = robot-to-sensor forward offset (robot frame +x = forward)
  - rss = robot-to-sensor sideways offset (robot frame +y = left)
  - Ground-truth positions measured from red-right corner, converted to WPILib field meters
"""

import subprocess, json, math, os
import numpy as np

CS        = r"C:/Users/frc68/.claude/plugins/cache/thefrc-suite/thefrc-suite/df63d24d0d2a/ClaudeScope/ClaudeScope.exe"
FMAP_PATH = r"C:\FRC\2026\Code\2026-CompBot\other\LL\FRC2026_WELDED.fmap"
HALF_LEN  = 16.541 / 2
HALF_WID  =  8.014 / 2

SPOTS = {
    "Spot1": r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot1.wpilog",
    "Spot2": r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot2.wpilog",
    "Spot3": r"C:\Users\Public\Documents\FRC\Log Files\2026\13_shop\spot3.wpilog",
}

GROUND_TRUTH = {
    "Spot1": (14.534, 5.436),
    "Spot2": (14.973, 2.553),
    "Spot3": (13.982, 3.245),
}

# (t0_s, t1_s, expected_LL3G_tags, expected_LL4_tags)
# Windows with empty expected lists are skipped — no botpose data expected.
WINDOWS = {
    "Spot1": {
        180: (169, 189, [13, 14],    [15]),
        270: (205, 228, [],          []),
          0: (251, 265, [9, 10, 11], [2, 11]),
         90: (275, 288, [15],        []),
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

# Current configured offsets from .vpr files (authoritative — VisionConstants.java comments are wrong).
# LL3G-shop.vpr → limelight-threeg   LL4-shop.vpr → limelight-four
CURRENT_OFFSETS = {
    "limelight-threeg": {"rsf": -0.27, "rss":  0.27},
    "limelight-four":   {"rsf": -0.29, "rss": -0.27},
}
CURRENT_RSYAW = {
    "limelight-threeg": -120.0,   # degrees, from LL3G-shop.vpr
    "limelight-four":    180.0,   # degrees, from LL4-shop.vpr
}

# ── Tag table ─────────────────────────────────────────────────────────────────

def load_tag_table():
    with open(FMAP_PATH) as f:
        fmap = json.load(f)
    tags = {}
    for fid in fmap.get("fiducials", []):
        t = fid["transform"]
        tags[fid["id"]] = {"x": t[3] + HALF_LEN, "y": t[7] + HALF_WID}
    return tags

TAG_TABLE = load_tag_table()

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

# ── Data queries ──────────────────────────────────────────────────────────────

def get_mean_heading_rad(sid, t0_us, t1_us):
    """Return circular mean of robot heading (radians) during the window."""
    raw = cs_range(sid, "/RealOutputs/RobotPose/Drive/rotation/value", t0_us, t1_us)
    vals = [s["value"] for s in (raw or [])
            if isinstance(s.get("value"), (int, float)) and int(s["timestamp"]) >= t0_us]
    if not vals:
        return None
    sin_sum = sum(math.sin(v) for v in vals)
    cos_sum = sum(math.cos(v) for v in vals)
    return math.atan2(sin_sum, cos_sum)

def get_botpose_mean(sid, cam, t0_us, t1_us):
    """
    Return (mean_x, mean_y, n_samples, mt_type) for the camera in the window.
    Tries MegaTag2 first (gyro-seeded, best for position calibration),
    then falls back to MegaTag1.  Returns None if neither has data.

    Only counts samples whose timestamps fall within [t0_us, t1_us] to avoid
    carrying in stale AKit hold-last-values from before the window.
    """
    for mt_label, base in [
        ("MT2", f"/RealOutputs/Vision/{cam}/MegaTag2/RobotPose"),
        ("MT1", f"/RealOutputs/Vision/{cam}/MegaTag1/RobotPose"),
    ]:
        len_raw = cs_range(sid, f"{base}/length", t0_us, t1_us)
        if not len_raw:
            continue
        len_in_win = [int(float(s["value"])) for s in len_raw
                      if isinstance(s.get("value"), (int, float))
                      and int(s["timestamp"]) >= t0_us]
        if max(len_in_win, default=0) == 0:
            continue

        x_raw = cs_range(sid, f"{base}/0/translation/x", t0_us, t1_us)
        y_raw = cs_range(sid, f"{base}/0/translation/y", t0_us, t1_us)

        xs = [float(s["value"]) for s in (x_raw or [])
              if isinstance(s.get("value"), (int, float)) and int(s["timestamp"]) >= t0_us]
        ys = [float(s["value"]) for s in (y_raw or [])
              if isinstance(s.get("value"), (int, float)) and int(s["timestamp"]) >= t0_us]

        if not xs or not ys:
            continue

        return (sum(xs) / len(xs), sum(ys) / len(ys), len(xs), mt_label)

    return None

# ── rsyaw Jacobian ────────────────────────────────────────────────────────────

def rsyaw_jacobian(cam, theta, gt_x, gt_y, exp_tags):
    """
    Partial derivatives of botpose (x, y) w.r.t. Δrsyaw (radians).

    When rsyaw is off by Δψ the camera misperceives each tag's field direction by −Δψ,
    shifting the computed camera position by d·Δψ·[−sin α, +cos α] per tag.

    Returns (Jψ_x, Jψ_y) averaged over the expected tags.
    Camera position is estimated from ground truth + current nominal offsets.
    """
    cur = CURRENT_OFFSETS[cam]
    c, s = math.cos(theta), math.sin(theta)
    cam_x = gt_x + cur["rsf"] * c - cur["rss"] * s
    cam_y = gt_y + cur["rsf"] * s + cur["rss"] * c

    jx_vals, jy_vals = [], []
    for tid in exp_tags:
        tp = TAG_TABLE.get(tid)
        if tp is None:
            continue
        dx = tp["x"] - cam_x
        dy = tp["y"] - cam_y
        d  = math.hypot(dx, dy)
        alpha = math.atan2(dy, dx)
        jx_vals.append(-d * math.sin(alpha))
        jy_vals.append( d * math.cos(alpha))

    if not jx_vals:
        return 0.0, 0.0
    return sum(jx_vals) / len(jx_vals), sum(jy_vals) / len(jy_vals)

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    all_obs = {"limelight-threeg": [], "limelight-four": []}

    for spot in ["Spot1", "Spot2", "Spot3"]:
        print(f"\nLoading {spot}...")
        sid = cs_load(SPOTS[spot])
        print(f"  session: {sid[:8]}...")
        gt_x, gt_y = GROUND_TRUTH[spot]

        for yaw in YAW_ORDER:
            t0_s, t1_s, exp_3g, exp_4 = WINDOWS[spot][yaw]
            t0_us = int(t0_s * 1_000_000)
            t1_us = int(t1_s * 1_000_000)

            theta = get_mean_heading_rad(sid, t0_us, t1_us)
            if theta is None:
                theta = math.radians(yaw)
                print(f"  WARNING: no heading data for {spot} {yaw}°, using nominal")

            # Only use windows where each camera sees >= 1 tag
            # AND at least one camera sees >= 2 tags
            if not (len(exp_3g) >= 1 and len(exp_4) >= 1):
                continue
            if not (len(exp_3g) >= 2 or len(exp_4) >= 2):
                continue

            for cam, exp_tags in [("limelight-threeg", exp_3g), ("limelight-four", exp_4)]:

                result = get_botpose_mean(sid, cam, t0_us, t1_us)
                if result is None:
                    print(f"  WARNING: no botpose for {cam} {spot} {yaw}° "
                          f"(expected tags {exp_tags})")
                    continue

                mx, my, n, mt_label = result
                ex = mx - gt_x
                ey = my - gt_y
                cam_short = "LL3G" if "threeg" in cam else "LL4 "
                print(f"  {cam_short} {spot} {yaw:>3}° [{mt_label}]  "
                      f"mean=({mx:.4f}, {my:.4f})  err=({ex:+.4f}, {ey:+.4f})  n={n}")

                all_obs[cam].append({
                    "spot": spot, "yaw": yaw, "theta": theta,
                    "err_x": ex, "err_y": ey, "n": n, "mt_type": mt_label,
                    "exp_tags": exp_tags,
                    "mean_x": mx, "mean_y": my,
                })

    # ── Per-spot summary (all windows, showing skipped ones too) ─────────────
    print(f"\n\n{'='*100}")
    print("PER-SPOT WINDOW SUMMARY  (all windows, skipped = no expected tags)")
    print(f"{'='*100}")
    hdr = (f"  {'Spot':<6} {'Yaw':>4}  {'Window(s)':>14}  "
           f"{'LL3G tags':>12}  {'LL3G botpose(x,y)':>22}  {'err(x,y)':>17}  "
           f"{'LL4 tags':>12}  {'LL4 botpose(x,y)':>22}  {'err(x,y)':>17}")
    print(hdr)
    print(f"  {'-'*97}")

    obs_lookup = {}
    for cam in ["limelight-threeg", "limelight-four"]:
        for o in all_obs[cam]:
            obs_lookup[(cam, o["spot"], o["yaw"])] = o

    for spot in ["Spot1", "Spot2", "Spot3"]:
        gt_x, gt_y = GROUND_TRUTH[spot]
        for yaw in YAW_ORDER:
            t0_s, t1_s, exp_3g, exp_4 = WINDOWS[spot][yaw]
            win_str = f"{t0_s}-{t1_s}s"

            pair_ok = (len(exp_3g) >= 1 and len(exp_4) >= 1 and
                       (len(exp_3g) >= 2 or len(exp_4) >= 2))

            def fmt_cam(cam, exp_tags):
                key = (cam, spot, yaw)
                if not exp_tags:
                    return f"{'(skip)':>12}  {'':>22}  {'':>17}"
                tag_str = str(exp_tags)
                if not pair_ok:
                    suffix = "" if len(exp_tags) >= 2 else " (1)"
                    return f"{tag_str+suffix:>12}  {'(skip-pair)':>22}  {'':>17}"
                o = obs_lookup.get(key)
                if o is None:
                    return f"{tag_str:>12}  {'NO DATA':>22}  {'':>17}"
                bp = f"({o['mean_x']:.3f},{o['mean_y']:.3f})"
                er = f"({o['err_x']:+.3f},{o['err_y']:+.3f})"
                return f"{tag_str:>12}  {bp:>22}  {er:>17}"

            ll3g_str = fmt_cam("limelight-threeg", exp_3g)
            ll4_str  = fmt_cam("limelight-four",   exp_4)
            print(f"  {spot:<6} {yaw:>4}  {win_str:>14}  {ll3g_str}  {ll4_str}")

    # ── Least-squares solve per camera ────────────────────────────────────────
    print(f"\n\n{'='*80}")
    print("CAMERA OFFSET CALIBRATION RESULTS")
    print(f"{'='*80}")

    for cam in ["limelight-threeg", "limelight-four"]:
        obs = all_obs[cam]
        print(f"\n{'-'*80}")
        print(f"Camera: {cam}  ({len(obs)} windows used)")

        if len(obs) < 2:
            print("  SKIPPED -- fewer than 2 observations")
            continue

        # Build system: [cos θ, -sin θ, Jψ_x; sin θ, cos θ, Jψ_y] * [drsf; drss; Δrsyaw] = [err_x; err_y]
        A_rows, b_rows = [], []
        for o in obs:
            c = math.cos(o["theta"])
            s = math.sin(o["theta"])
            gt_x_o, gt_y_o = GROUND_TRUTH[o["spot"]]
            jx, jy = rsyaw_jacobian(cam, o["theta"], gt_x_o, gt_y_o, o["exp_tags"])
            A_rows += [[c, -s, jx], [s,  c, jy]]
            b_rows += [o["err_x"], o["err_y"]]

        A = np.array(A_rows)
        b = np.array(b_rows)
        (drsf, drss, drsyaw_rad), _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        drsyaw_deg = math.degrees(drsyaw_rad)

        cur     = CURRENT_OFFSETS[cam]
        cur_yaw = CURRENT_RSYAW[cam]
        new_rsf   = cur["rsf"] + drsf
        new_rss   = cur["rss"] + drss
        new_rsyaw = cur_yaw    + drsyaw_deg

        pred  = A @ np.array([drsf, drss, drsyaw_rad])
        rms_m = math.sqrt(np.mean((b - pred) ** 2))

        print(f"  Current:     rsf={cur['rsf']:+.5f} m   rss={cur['rss']:+.5f} m   rsyaw={cur_yaw:+.2f} deg")
        print(f"  Correction:  drsf={drsf:+.5f} m   drss={drss:+.5f} m   Drsyaw={drsyaw_deg:+.2f} deg")
        print(f"  Recommended: rsf={new_rsf:+.5f} m   rss={new_rss:+.5f} m   rsyaw={new_rsyaw:+.2f} deg")
        print(f"  RMS residual: {rms_m*100:.1f} cm")

        # Per-observation breakdown
        print(f"\n  {'Spot':<6} {'Yaw':>4} {'Type':>4} {'Tags':<14} {'n':>5}  "
              f"{'err_x':>7} {'err_y':>7}  {'fit_x':>7} {'fit_y':>7}  {'res':>6}")
        for o in obs:
            c = math.cos(o["theta"])
            s = math.sin(o["theta"])
            gt_x_o, gt_y_o = GROUND_TRUTH[o["spot"]]
            jx, jy = rsyaw_jacobian(cam, o["theta"], gt_x_o, gt_y_o, o["exp_tags"])
            fit_x = drsf * c - drss * s + drsyaw_rad * jx
            fit_y = drsf * s + drss * c + drsyaw_rad * jy
            res_m = math.hypot(o["err_x"] - fit_x, o["err_y"] - fit_y)
            tag_str = str(o["exp_tags"])
            print(f"  {o['spot']:<6} {o['yaw']:>4} {o['mt_type']:>4} {tag_str:<14} {o['n']:>5}  "
                  f"{o['err_x']:>+7.4f} {o['err_y']:>+7.4f}  "
                  f"{fit_x:>+7.4f} {fit_y:>+7.4f}  {res_m*100:>5.1f}cm")

    print()


if __name__ == "__main__":
    main()
