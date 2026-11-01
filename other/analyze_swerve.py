#!/usr/bin/env python3
"""
Swerve module health analyzer for 2026-CompBot.

Requires: Python 3.8+, ClaudeScope.exe (from thefrc-suite plugin)

Usage:
    python analyze_swerve.py <path_to_wpilog>
    python analyze_swerve.py <path_to_wpilog> --cs <path_to_ClaudeScope.exe>

Examples:
    python analyze_swerve.py "C:/Users/Public/Documents/FRC/Log Files/2026/12_BGRC/akit_q20.wpilog"
    python analyze_swerve.py mylog.wpilog --cs C:/tools/ClaudeScope.exe
"""

import io
import json
import os
import statistics
import subprocess
import sys
from pathlib import Path

# Force UTF-8 output on Windows so any Unicode in log paths prints cleanly
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

# ── Configuration ─────────────────────────────────────────────────────────────

# Known install locations for ClaudeScope.exe — first found wins
CS_SEARCH_PATHS = [
    r"C:\Users\frc68\.claude\plugins\cache\thefrc-suite\thefrc-suite\df63d24d0d2a\ClaudeScope\ClaudeScope.exe",
]

MODULE_NAMES = ["FL (Front Left)", "FR (Front Right)", "BL (Back Left)", "BR (Back Right)"]
MODULE_KEYS  = ["FrontLeft", "FrontRight", "BackLeft", "BackRight"]

# ── ClaudeScope helpers ───────────────────────────────────────────────────────

def find_claudescope():
    for path in CS_SEARCH_PATHS:
        if os.path.exists(path):
            return path
    # Auto-search the plugin cache (works regardless of version hash in path)
    cache = Path.home() / ".claude" / "plugins" / "cache"
    if cache.exists():
        for exe in cache.rglob("ClaudeScope.exe"):
            return str(exe)
    return None


def cs_run(cs_path, *args):
    env = os.environ.copy()
    env["MSYS_NO_PATHCONV"] = "1"
    result = subprocess.run(
        [cs_path, *args],
        capture_output=True, text=True, env=env,
    )
    if result.returncode != 0:
        raise RuntimeError(f"ClaudeScope failed: {result.stderr or result.stdout}")
    return json.loads(result.stdout)


def cs_load(cs_path, log_path):
    data = cs_run(cs_path, "load", log_path)
    sid = data.get("session_id")
    if not sid:
        raise RuntimeError(f"Load failed: {data}")
    return sid


def cs_range(cs_path, sid, *fields):
    return cs_run(cs_path, "range", *fields, "--session", sid, "--start", "0", "--end", "0")

# ── Analysis helpers ──────────────────────────────────────────────────────────

def steady_state_ratios(setpts_raw, meas_raw, min_speed=1.0):
    """Return list of (measured/setpoint) ratios for steady-state samples."""
    setpts = sorted(setpts_raw, key=lambda x: x["timestamp"])
    meas   = sorted(meas_raw,   key=lambda x: x["timestamp"])
    ratios = []
    for i in range(2, len(setpts)):
        s0, s1, s2 = setpts[i-2], setpts[i-1], setpts[i]
        v = s2["value"]
        if abs(v) < min_speed:
            continue
        if abs(s1["value"] - v) / abs(v) > 0.05 or abs(s0["value"] - v) / abs(v) > 0.05:
            continue
        t = s2["timestamp"]
        nearest = min(meas, key=lambda x: abs(x["timestamp"] - t), default=None)
        if nearest and abs(nearest["timestamp"] - t) < 60_000:
            ratios.append(nearest["value"] / v)
    return ratios


def mean_safe(lst):
    return statistics.mean(lst) if lst else None


def undershoot_label(pct):
    if pct <= 5:   return "excellent"
    if pct <= 15:  return "acceptable"
    if pct <= 25:  return "poor"
    return "CRITICAL"

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    # Parse CLI arguments
    log_path = None
    cs_path  = None
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--cs" and i + 1 < len(args):
            cs_path = args[i + 1]; i += 2
        else:
            log_path = args[i]; i += 1

    if not log_path:
        print(__doc__)
        sys.exit(1)

    if not cs_path:
        cs_path = find_claudescope()
    if not cs_path or not os.path.exists(cs_path):
        print("ERROR: ClaudeScope.exe not found.")
        print("Install it via the thefrc-suite Claude Code plugin, or pass --cs <path>.")
        sys.exit(1)

    print(f"Log:         {log_path}")
    print(f"ClaudeScope: {cs_path}")
    print("Loading log...")

    sid = cs_load(cs_path, log_path)
    print(f"Session:     {sid}\n")

    # ── Gains ─────────────────────────────────────────────────────────────────
    print("Querying gains, battery, velocity, current...")

    gain_data = cs_range(cs_path, sid,
        "/Tuning/Drive/Module/DrivekV",
        "/Tuning/Drive/Module/DrivekS",
        "/Tuning/Drive/Module/DrivekP",
    )
    kv_samples = sorted(gain_data.get("/Tuning/Drive/Module/DrivekV") or [], key=lambda x: x["timestamp"])
    ks_samples = gain_data.get("/Tuning/Drive/Module/DrivekS") or []
    kp_samples = gain_data.get("/Tuning/Drive/Module/DrivekP") or []
    kv_changed  = len(kv_samples) > 1
    kv_change_ts = kv_samples[-1]["timestamp"] if kv_changed else None
    kv_final = kv_samples[-1]["value"] if kv_samples else None
    ks_val   = ks_samples[-1]["value"] if ks_samples else None
    kp_val   = kp_samples[-1]["value"] if kp_samples else None

    # ── Battery ───────────────────────────────────────────────────────────────
    batt_data = cs_range(cs_path, sid, "/SystemStats/BatteryVoltage")
    batt_vals = [b["value"] for b in (batt_data.get("/SystemStats/BatteryVoltage") or [])]
    batt_mean = mean_safe(batt_vals)
    batt_min  = min(batt_vals) if batt_vals else None

    # ── Velocity tracking ─────────────────────────────────────────────────────
    vel_fields = []
    for n in range(4):
        vel_fields += [
            f"/RealOutputs/SwerveStates/Setpoints/{n}/speed",
            f"/RealOutputs/SwerveStates/Measured/{n}/speed",
        ]
    vel_data = cs_range(cs_path, sid, *vel_fields)

    # Build per-module ratios (split by kV change if it occurred)
    mod_ratios        = {}   # full period (or only period if no change)
    mod_ratios_before = {}
    mod_ratios_after  = {}
    for n in range(4):
        sp = vel_data.get(f"/RealOutputs/SwerveStates/Setpoints/{n}/speed") or []
        me = vel_data.get(f"/RealOutputs/SwerveStates/Measured/{n}/speed")  or []
        if kv_changed and kv_change_ts:
            mod_ratios_before[n] = steady_state_ratios(
                [s for s in sp if s["timestamp"] <  kv_change_ts],
                [m for m in me if m["timestamp"] <  kv_change_ts])
            mod_ratios_after[n]  = steady_state_ratios(
                [s for s in sp if s["timestamp"] >= kv_change_ts],
                [m for m in me if m["timestamp"] >= kv_change_ts])
        else:
            mod_ratios[n] = steady_state_ratios(sp, me)

    # ── Drive supply current (only while driving) ─────────────────────────────
    cur_fields = [f"/Drive/Module{n}({MODULE_KEYS[n]})/DriveSupplyCurrentAmps" for n in range(4)]
    cur_data = cs_range(cs_path, sid, *cur_fields)

    # Build set of 50ms time buckets when any module was commanded > 1.5 m/s
    driving_buckets = set()
    for n in range(4):
        for s in (vel_data.get(f"/RealOutputs/SwerveStates/Setpoints/{n}/speed") or []):
            if abs(s["value"]) > 1.5:
                driving_buckets.add(s["timestamp"] // 50_000)

    mod_current = {}
    for n in range(4):
        key = f"/Drive/Module{n}({MODULE_KEYS[n]})/DriveSupplyCurrentAmps"
        active = [s["value"] for s in (cur_data.get(key) or [])
                  if s["timestamp"] // 50_000 in driving_buckets]
        mod_current[n] = mean_safe(active)

    # ── Steer supply current ──────────────────────────────────────────────────
    turn_fields = [f"/Drive/Module{n}({MODULE_KEYS[n]})/TurnSupplyCurrentAmps" for n in range(4)]
    turn_data = cs_range(cs_path, sid, *turn_fields)

    mod_steer_peak   = {}
    mod_steer_spikes = {}
    for n in range(4):
        key  = f"/Drive/Module{n}({MODULE_KEYS[n]})/TurnSupplyCurrentAmps"
        vals = [abs(s["value"]) for s in (turn_data.get(key) or [])]
        mod_steer_peak[n]   = max(vals) if vals else 0.0
        mod_steer_spikes[n] = sum(1 for v in vals if v > 20)

    min_spikes = min(mod_steer_spikes.values()) if mod_steer_spikes else 1

    # ─────────────────────────────────────────────────────────────────────────
    # REPORT
    # ─────────────────────────────────────────────────────────────────────────
    W = 60
    sep = "─" * W

    def section(title):
        print(f"\n-- {title} {'-' * max(0, W - len(title) - 4)}")

    print()
    print("=" * W)
    print("  SWERVE HEALTH REPORT")
    print(f"  {os.path.basename(log_path)}")
    print("=" * W)

    # Gains
    section("ACTIVE GAINS")
    if kv_samples:
        if kv_changed:
            vals = [s["value"] for s in kv_samples]
            print(f"  kV: {vals[0]} -> {vals[-1]}  (changed at {kv_change_ts/1e6:.1f}s)")
        else:
            print(f"  kV: {kv_final}")
    else:
        print("  kV: not found in log")
    print(f"  kS: {ks_val if ks_val is not None else 'not found'}")
    print(f"  kP: {kp_val if kp_val is not None else 'not found'}")

    # Battery
    section("BATTERY")
    if batt_mean is not None:
        if   batt_min >= 8.0: status = "OK"
        elif batt_min >= 7.0: status = "WARNING — test battery capacity"
        else:                  status = "CRITICAL — REPLACE BEFORE COMPETITION"
        print(f"  Mean: {batt_mean:.2f} V    Min: {batt_min:.2f} V    [{status}]")
    else:
        print("  No battery data")

    # Velocity tracking
    section("VELOCITY TRACKING  (setpoint vs measured)")

    def print_tracking(ratios_dict, label=None):
        if label:
            print(f"  [{label}]")
        print(f"  {'Module':<20} {'n':>5}   {'Undershoot':>10}   Status")
        pcts = []
        for n, name in enumerate(MODULE_NAMES):
            r = ratios_dict.get(n, [])
            if r:
                pct = (1 - statistics.mean(r)) * 100
                pcts.append(pct)
                print(f"  {name:<20} {len(r):>5}   {pct:>9.1f}%   {undershoot_label(pct)}")
            else:
                print(f"  {name:<20} {'—':>5}   {'no data':>10}")
        return pcts

    if kv_changed:
        kv_vals = [s["value"] for s in kv_samples]
        print_tracking(mod_ratios_before, label=f"kV = {kv_vals[0]}")
        print()
        pcts_after = print_tracking(mod_ratios_after, label=f"kV = {kv_vals[-1]}")
        mean_under = mean_safe(pcts_after) or 0
        kv_suggested = round(kv_vals[-1] + mean_under / 100 * 0.7, 2)
    else:
        pcts_all = print_tracking(mod_ratios)
        mean_under = mean_safe(pcts_all) or 0
        kv_suggested = round((kv_final or 0) + mean_under / 100 * 0.7, 2)

    if mean_under > 5:
        print(f"\n  Systematic undershoot: {mean_under:.1f}%  ->  suggested next kV: {kv_suggested}")

    # Drive current
    section("DRIVE CURRENT  (active driving only)")
    valid_cur = {n: v for n, v in mod_current.items() if v is not None}
    if valid_cur:
        fleet_avg = statistics.mean(valid_cur.values())
        for n, name in enumerate(MODULE_NAMES):
            c = mod_current.get(n)
            if c is None:
                print(f"  {name:<20}  no data")
                continue
            flag = ""
            if c < fleet_avg - 2.0:
                flag = "  ← LOW — suspected mechanical drag"
            elif c > fleet_avg + 3.0:
                flag = "  ← HIGH"
            print(f"  {name:<20}  {c:>5.1f} A  (fleet avg {fleet_avg:.1f} A){flag}")
    else:
        print("  No drive current data")

    # Steer current
    section("STEER CURRENT  (friction / stiction)")
    for n, name in enumerate(MODULE_NAMES):
        peak   = mod_steer_peak.get(n, 0)
        spikes = mod_steer_spikes.get(n, 0)
        flag = ""
        if peak > 60 or (min_spikes > 0 and spikes > min_spikes * 3):
            flag = "  ← STICTION — regrease bearing, check bevel preload"
        elif peak > 40:
            flag = "  ← elevated — monitor"
        print(f"  {name:<20}  peak {peak:>5.1f} A   spikes(>20A): {spikes:>4}{flag}")

    # Action items
    section("ACTION ITEMS")
    items = []

    if batt_min is not None and batt_min < 7.0:
        items.append("BATTERY: Replace before next competition (min < 7.0 V)")
    elif batt_min is not None and batt_min < 8.0:
        items.append("BATTERY: Test capacity, consider replacement (min < 8.0 V)")

    if mean_under > 5:
        kv_now = (kv_vals[-1] if kv_changed else kv_final) or 0
        items.append(f"TUNING:  Increase drivekV {kv_now} -> {kv_suggested} in Module.java (or via tuning mode)")

    items.append("CODE:    Verify wheelRadius = Units.inchesToMeters(1.914) in DriveConstants.java:29")

    if valid_cur:
        fleet_avg = statistics.mean(valid_cur.values())
        for n, name in enumerate(MODULE_NAMES):
            c = mod_current.get(n)
            if c is not None and c < fleet_avg - 2.0:
                items.append(f"MECH:    {name} — low drive current, inspect belt / bearings / bevel gear")

    for n, name in enumerate(MODULE_NAMES):
        peak   = mod_steer_peak.get(n, 0)
        spikes = mod_steer_spikes.get(n, 0)
        if peak > 60 or (min_spikes > 0 and spikes > min_spikes * 3):
            items.append(f"MECH:    {name} — steer stiction, regrease bearing and check bevel preload")

    for idx, item in enumerate(items, 1):
        print(f"  {idx}. {item}")

    print(f"\n{'=' * W}\n")


if __name__ == "__main__":
    main()
