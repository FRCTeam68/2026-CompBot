# Swerve Module Health Analysis

Analyze swerve drive performance from a `.wpilog` file.
Log path: `$ARGUMENTS`

Follow these steps exactly, in order.

---

## Step 1 — Load the log

```
MSYS_NO_PATHCONV=1 "C:/Users/frc68/.claude/plugins/cache/thefrc-suite/thefrc-suite/df63d24d0d2a/ClaudeScope/ClaudeScope.exe" load "<log_path>"
```

Extract `session_id` from the response. All subsequent ClaudeScope calls use `--session <id>`.

---

## Step 2 — Check tuning gains that were actually active

Query `/Tuning/Drive/Module/DrivekV`, `/Tuning/Drive/Module/DrivekS`, `/Tuning/Drive/Module/DrivekP` with `--start 0 --end 0`.

Report the values (or note if they changed mid-log and at what timestamp). These are what Phoenix was actually using, independent of what is in code.

---

## Step 3 — Battery health

Query `/SystemStats/BatteryVoltage` with `--start 0 --end 0`.

Report: mean voltage, minimum voltage, and whether the minimum is below 8.0V (warning) or below 7.0V (critical — replace battery before competition).

---

## Step 4 — Velocity tracking (setpoint vs measured) per module

Query all 8 fields at once with `--start 0 --end 0`:
- `/RealOutputs/SwerveStates/Setpoints/0/speed` through `...3/speed`
- `/RealOutputs/SwerveStates/Measured/0/speed` through `...3/speed`

For each module (FL=0, FR=1, BL=2, BR=3), compute **steady-state tracking ratio**:
- Find consecutive setpoint samples where |setpoint| > 1.0 m/s and the value is stable (3+ consecutive samples within 5% of each other).
- Match each to the nearest measured sample within 60ms.
- Compute `ratio = measured / setpoint`.
- Report: sample count, mean ratio, and undershoot % = `(1 - ratio) * 100`.

**Interpretation guide:**
- 0–5% undershoot: excellent — kV is well-tuned
- 5–15% undershoot: acceptable — consider kV increase
- 15–25% undershoot: poor — kV too low or significant mechanical drag
- >25% undershoot: critical — check kV=0 or major mechanical problem

**If kV changed mid-log**, split the analysis into before/after periods and report both.

---

## Step 5 — Drive motor current asymmetry (mechanical drag detection)

Query `/Drive/Module0(FrontLeft)/DriveSupplyCurrentAmps`, `Module1(FrontRight)`, `Module2(BackLeft)`, `Module3(BackRight)` with `--start 0 --end 0`.

For each module, compute mean supply current only during periods when any setpoint speed > 1.5 m/s (robot is actually being driven).

Report mean current for each module. Flag any module whose mean is more than 2A below the average of the other three as **low current / suspected mechanical drag** (low current = wheel spinning slower due to friction binding elsewhere in drivetrain).

---

## Step 6 — Steer torque current spikes (steer friction / stiction)

Query `/Drive/Module0(FrontLeft)/TurnSupplyCurrentAmps` through `Module3` with `--start 0 --end 0`.

For each module compute:
- Peak steer supply current
- Count of samples where steer current > 20A

Flag any module with peak > 60A or >3× as many high-current events as the module with the least as having **steer friction / stiction** (likely symptom: stiction breakaway spikes; check steer bearing grease and bevel gear preload).

---

## Step 7 — Report

Output a structured report with these sections:

### Active Gains
List the kV/kS/kP values confirmed from the log.

### Battery
Status (OK / Warning / Critical), mean/min voltages.

### Velocity Tracking Summary
Table: Module | Undershoot % | Status
Plus: recommended kV adjustment if systematic undershoot > 5% across all modules.
Formula for next kV estimate:
```
undershoot_fraction = mean undershoot / 100
kV_next = kV_current + (kP * undershoot_fraction * max_wheel_rps) / max_wheel_rps
# simplified: kV_next ≈ kV_current + (undershoot_fraction * 0.7)
```
Where max_wheel_rps ≈ 15.76 (for this robot: 5.03 m/s / 0.0486 m wheel radius / 2π).

### Mechanical Health
Per module: current asymmetry finding, steer spike finding.
Flag which modules need physical inspection.

### Action Items
Numbered list, ordered by impact:
1. Battery replacement if critical
2. kV adjustment with specific recommended value
3. Module-specific mechanical repairs
4. Wheel radius code fix if not yet applied (correct value: `Units.inchesToMeters(1.914)` in DriveConstants.java line 29)
