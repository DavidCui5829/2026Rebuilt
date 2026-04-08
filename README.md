# 2026 Rebuilt - AdvantageKit Logging Reference

Quick reference for all `Logger.recordOutput()` keys. Open these in AdvantageScope to troubleshoot.

---

## Drive (SwerveSubsystem)

### Odometry & Pose

| Key | Type | What it tells you |
|---|---|---|
| `Drive/Pose` | Pose2d | Current robot pose from odometry |
| `Odometry/Robot` | Pose2d | Same as Drive/Pose (duplicate for AdvantageScope field widget) |
| `Drive/Heading` | Rotation2d | Current robot heading |
| `Drive/OdometryHeading` | Rotation2d | Heading from the pose (should match Drive/Heading) |
| `Drive/GyroYaw` | Rotation2d | Raw gyro yaw |
| `Drive/GyroPitch` | Rotation2d | Raw gyro pitch |
| `Drive/GyroRoll` | Rotation2d | Raw gyro roll |
| `Drive/GyroRotation3d` | Rotation3d | Full 3D gyro orientation |

### Velocity

| Key | Type | What it tells you |
|---|---|---|
| `Drive/RobotVelocity` | ChassisSpeeds | Measured velocity in robot frame |
| `Drive/FieldVelocity` | ChassisSpeeds | Measured velocity in field frame |
| `Drive/CommandedRobotVelocity` | ChassisSpeeds | What we told the drivebase to do (robot frame) |
| `Drive/CommandedFieldVelocity` | ChassisSpeeds | What we told the drivebase to do (field frame) |
| `Drive/CommandedOmega` | double | Commanded rotational rate (rad/s). Should be ~0 with no turn input |
| `Drive/ActualOmega` | double | Measured rotational rate (rad/s). Compare with commanded to check responsiveness |

### Module States

| Key | Type | What it tells you |
|---|---|---|
| `Drive/CurrentModuleStates` | SwerveModuleState[] | Actual speed + angle of each module |
| `Drive/DesiredModuleStates` | SwerveModuleState[] | What we commanded each module to do |
| `Drive/ModulePositions` | SwerveModulePosition[] | Drive distance + angle for each module |

### Aim - Hub (SOTM)

| Key | Type | What it tells you |
|---|---|---|
| `Drive/Aim/DynamicHubPose` | Pose2d | Velocity-compensated hub aim point. Overlay on field to see SOTM lead |
| `Drive/Aim/StaticHubPose` | Pose2d | Actual hub position. Compare with dynamic to see how much lead is applied |
| `Drive/Aim/TargetAngleDeg` | double | Desired robot heading to face the compensated hub |
| `Drive/Aim/CurrentHeadingDeg` | double | Actual robot heading. Compare with target to sanity check error |
| `Drive/Aim/ErrorDegHub` | double | Degrees off from hub aim target. Should converge to ~0 when aiming. If not, PID tuning issue |
| `Drive/Aim/DistanceToHubM` | double | Distance to compensated hub. Correlate with shot accuracy to validate TOF table |
| `Drive/Aim/RobotVelX` | double | Field-relative X velocity (m/s). If SOTM lead looks wrong, check for noisy velocity |
| `Drive/Aim/RobotVelY` | double | Field-relative Y velocity (m/s) |
| `Drive/Aim/IsLocked` | boolean | If true, SOTM is bypassed and static hub pose is used |

### Aim - Ferry

| Key | Type | What it tells you |
|---|---|---|
| `Drive/Aim/DynamicFerryPose` | Pose2d | Velocity-compensated ferry aim point. Overlay on field to verify correct ferry spot |
| `Drive/Aim/ErrorDegFerry` | double | Degrees off from ferry aim target. Same use as hub error |
| `Drive/Aim/DistanceToFerryM` | double | Distance to ferry target. Useful if passes fall short or overshoot |

### Autonomous Recovery

| Key | Type | What it tells you |
|---|---|---|
| `Drive/Auto/TargetPathPose` | Pose2d | Where PathPlanner wants the robot to be right now |
| `Drive/Auto/PathFollowingErrorM` | double | Distance (m) between robot and path target. Graph to see when error spikes |
| `Drive/Auto/IsOffPath` | boolean | True when error > 0.15m. This is when `followWithRecovery` switches to recovery path |

---

## Shooter

| Key | Type | What it tells you |
|---|---|---|
| `Shooter/Right1RPM` | double | Right flywheel 1 speed |
| `Shooter/Right2RPM` | double | Right flywheel 2 speed |
| `Shooter/Left1RPM` | double | Left flywheel 1 speed |
| `Shooter/Left2RPM` | double | Left flywheel 2 speed |
| `Shooter/TargetRPM` | double | Commanded flywheel RPM. Compare with actual to check spin-up time |
| `Shooter/TargetKickerRPM` | double | Commanded kicker RPM |
| `Shooter/Right1AppliedVolts` | double | Voltage applied to right motor 1 |
| `Shooter/Right2AppliedVolts` | double | Voltage applied to right motor 2 |
| `Shooter/Left1AppliedVolts` | double | Voltage applied to left motor 1 |
| `Shooter/Left2AppliedVolts` | double | Voltage applied to left motor 2 |
| `Shooter/KickerLeftRPM` | double | Left kicker actual speed |
| `Shooter/KickerRightRPM` | double | Right kicker actual speed |
| `Shooter/KickerLeftAppliedVolts` | double | Voltage applied to left kicker |
| `Shooter/KickerRightAppliedVolts` | double | Voltage applied to right kicker |
| `Shooter/LUTCurrentTargetRPM` | double | RPM from the lookup table for current distance |
| `Shooter/LUTDistance` | double | Distance used for the LUT lookup |

---

## Intake

| Key | Type | What it tells you |
|---|---|---|
| `Intake/DesiredPercent` | double | Commanded intake percent output |
| `Intake/AppliedVolts` | double | Actual voltage to intake motor |
| `IntakeRightRPM` | double | Right intake roller speed |
| `IntakeLeftRPM` | double | Left intake roller speed |
| `IntakeTargetRPM` | double | Target intake RPM from constants |

---

## Hopper

| Key | Type | What it tells you |
|---|---|---|
| `Hopper/PushdownDesiredPercent` | double | Commanded pushdown (right twindexer) percent |
| `Hopper/TransferDesiredPercent` | double | Commanded transfer (left twindexer) percent |
| `Hopper/TwindexerRightMotor` | double | Right twindexer applied output |
| `Hopper/TwindexerLeftMotor` | double | Left twindexer applied output |

---

## Pushout

| Key | Type | What it tells you |
|---|---|---|
| `Pushout/DesiredPercent` | double | Commanded pushout percent output |

---

## Object Detection

| Key | Type | What it tells you |
|---|---|---|
| `ObjectDetection/PipelineHasTargets` | boolean | Whether the camera sees any game pieces |
| `ObjectDetection/TargetPose` | Transform3d | Best camera-to-target transform |
| `ObjectDetection/AlignToFuelPose` | Pose2d | Computed pose to drive to for pickup |

---

## Shooting Sequence (RT Command)

These log the state of the RT shooting/passing sequence — the gates that control when feeding starts and stops.

| Key | Type | What it tells you |
|---|---|---|
| `Shooting/RTHeld` | boolean | Whether the driver right trigger is pressed |
| `Shooting/InAllianceZone` | boolean | Which branch was taken — true = shooting at hub, false = passing to ferry |
| `Shooting/AimLock1Deg` | boolean | True when aim error is within 1 deg of hub. This is the gate for feeding to start |
| `Shooting/AimLock3Deg` | boolean | True when aim error is within 3 deg of hub. Feeding stops if this goes false |
| `Shooting/FerryAimLock3Deg` | boolean | True when aim error is within 3 deg of ferry target. Gate for ferry feeding |

---

## Controller Input

### Driver

| Key | Type | What it tells you |
|---|---|---|
| `Input/Driver/LeftX` | double | Left stick X axis |
| `Input/Driver/LeftY` | double | Left stick Y axis |
| `Input/Driver/RightX` | double | Right stick X axis |
| `Input/Driver/RightY` | double | Right stick Y axis |
| `Input/Driver/LeftTrigger` | double | Left trigger axis (0-1) |
| `Input/Driver/RightTrigger` | double | Right trigger axis (0-1) |

### Operator

| Key | Type | What it tells you |
|---|---|---|
| `Input/Operator/LeftX` | double | Left stick X axis |
| `Input/Operator/LeftY` | double | Left stick Y axis |
| `Input/Operator/RightX` | double | Right stick X axis |
| `Input/Operator/RightY` | double | Right stick Y axis |
| `Input/Operator/LeftTrigger` | double | Left trigger axis (0-1) |
| `Input/Operator/RightTrigger` | double | Right trigger axis (0-1) |

---

## Important Notes

- **Aim logs only appear when RT is held** (`isAiming = true`). If you don't see `Drive/Aim/` keys updating, the aim command isn't running.
- **Auto recovery logs run always** during auto and teleop.
- All `Drive/` logs come from `SwerveSubsystem.periodic()`. All `Shooter/` logs come from `Shooter.periodic()`. Controller input logs come from `RobotContainer`.

---

## Troubleshooting Guide

### Drivetrain Issues

| Symptom | What to check | What it means |
|---|---|---|
| Robot drifting when driving straight | `Drive/CommandedOmega` should be ~0, check `Drive/ActualOmega` | If actual omega is nonzero with zero commanded, heading correction may be off or gyro is drifting |
| Robot not driving at full speed | `Drive/CommandedFieldVelocity` vs `Drive/FieldVelocity` | If commanded is low, check for `scaleTranslation`. If actual is low but commanded is high, mechanical issue |
| Odometry jumping | `Drive/Pose` on field view, check for sudden teleports | Vision measurement with bad data got accepted. Check Limelight rejection thresholds |
| Modules fighting each other | `Drive/CurrentModuleStates` vs `Drive/DesiredModuleStates` | If angles are far apart, module PID tuning issue or encoder offset wrong |

### Aiming / SOTM Issues

| Symptom | What to check | What it means |
|---|---|---|
| Aim lagging behind while moving | Check if `aimLookahead` is set in AimAtHub. Graph `ErrorDegHub` | Without lookahead, aim chases current position instead of leading. Should have 0.2s lookahead |
| Aim oscillating, never settling | Graph `ErrorDegHub` — if it bounces +/- degrees | Heading PID gains too aggressive, or `aimFeedforward` values need tuning |
| Aim not tracking at all | Check `Drive/Aim/DynamicHubPose` exists in log | If missing, `isAiming` is false — AimAtHub isn't running. Check command scheduling |
| SOTM lead looks wrong | Compare `DynamicHubPose` vs `StaticHubPose` on field view | Lead should point opposite to robot velocity. If not, check `RobotVelX`/`RobotVelY` for noisy readings |
| Aim stops when shooter reaches speed | Check if AimAtHub gets interrupted in command scheduler | `lockCommand` or another drivebase command may be taking over. AimAtHub should hold drivebase the entire time |
| Shots missing at distance | Graph `DistanceToHubM` and correlate with `Shooter/LUTDistance` | If they don't match, the shooter RPM lookup is using a different distance than the aim system |
| `IsLocked` is true unexpectedly | Check `Drive/Aim/IsLocked` | When locked, SOTM is bypassed and static hub pose is used. Robot must be stationary (no sticks) for this |

### Shooting Sequence Issues

| Symptom | What to check | What it means |
|---|---|---|
| Nothing happens when RT pressed | `Shooting/RTHeld` should be true | If false, trigger not registering. If true, check `Shooting/InAllianceZone` — may have taken wrong branch |
| Shooter spins but never feeds | `Shooting/AimLock1Deg` — is it ever true? | Feeding waits for shooter at speed AND aim within 1 deg. If aim never locks, check `ErrorDegHub` |
| Feeding starts then immediately stops | `Shooting/AimLock3Deg` — does it flicker? | Feeding has a 3 deg `onlyWhile` gate. If aim oscillates around 3 deg, feeding cuts in and out |
| Shooting when not aimed at hub | `Shooting/AimLock1Deg` should be false until aimed | If true when not aimed, the aimLock threshold may be too loose or the error calculation is wrong |
| Passing not working in opponent zone | `Shooting/InAllianceZone` should be false | If true, robot thinks it's in alliance zone. Check zone boundary thresholds (182in blue, 469in red) |
| Ferry feeding cuts out | `Shooting/FerryAimLock3Deg` — is it stable? | Same as hub — if aim oscillates near 3 deg, feeding stops. May need wider threshold |

### Shooter Motor Issues

| Symptom | What to check | What it means |
|---|---|---|
| Shooter not spinning up | `Shooter/TargetRPM` — is it nonzero? | If zero, the shoot command isn't running. If nonzero, check `AppliedVolts` — motor may not be responding |
| Shooter slow to reach speed | Graph `Shooter/TargetRPM` vs actual RPMs over time | Large gap = PID too slow, voltage sag, or mechanical drag |
| Left/Right shooter mismatch | Compare `Left1RPM`/`Left2RPM` vs `Right1RPM`/`Right2RPM` | If one side lags, check for mechanical binding or motor issue on that side |
| Kicker not feeding | `Shooter/KickerLeftRPM` and `KickerRightRPM` should be nonzero | If zero, kicker command isn't running. Check the shooting sequence composition |
| Shots inconsistent at same distance | `Shooter/LUTCurrentTargetRPM` should be stable | If RPM fluctuates, the distance reading is noisy — check `LUTDistance` |

### Intake / Hopper Issues

| Symptom | What to check | What it means |
|---|---|---|
| Intake not running | `Intake/DesiredPercent` — should be nonzero when commanded | If zero, command isn't reaching the intake. If nonzero but `AppliedVolts` is zero, motor issue |
| Fuel not transferring to shooter | `Hopper/TransferDesiredPercent` and `Hopper/PushdownDesiredPercent` | Both should be nonzero during the feed sequence |
| Fuel jamming | Check if `Pushout/DesiredPercent` is cycling (agitate) | If not, the agitate command may not be running in the shooting sequence |

### Auto Recovery Issues

| Symptom | What to check | What it means |
|---|---|---|
| Recovery triggering too early | Graph `PathFollowingErrorM` — what's the normal tracking error? | If it regularly hits 0.12-0.14m during normal runs, the 0.15m threshold is too tight. Increase to 0.2-0.25m |
| Recovery not triggering when knocked | `IsOffPath` should flip true when hit | If it stays false, the threshold is too loose or `targetPathPose` isn't updating (check `configurePathPlannerLogging`) |
| Recovery flickering | `IsOffPath` rapidly toggles true/false | Robot is hovering at the threshold boundary. Need hysteresis or a wider threshold |
| Robot recovers but ends up in wrong spot | Overlay `TargetPathPose` and `Drive/Pose` on field view | Recovery path may not end where the next auto step expects. Check recovery path endpoints |

### Controller Input Issues

| Symptom | What to check | What it means |
|---|---|---|
| Robot not responding to sticks | Check `Input/Driver/LeftX`, `LeftY`, `RightX` | If values change with sticks, the issue is downstream. If stuck at 0, controller not connected or wrong port |
| Wrong controller is driving | Compare `Input/Driver/` vs `Input/Operator/` values while pressing sticks | Controller may be assigned to wrong role. Check the driver/operator chooser |
