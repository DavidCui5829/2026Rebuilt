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

### Auto Recovery

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

## Troubleshooting Tips

- **Shots missing while moving**: Graph `Drive/Aim/ErrorDegHub` - if it oscillates instead of settling, heading PID needs tuning. Check `RobotVelX`/`RobotVelY` for noisy velocity readings that could throw off SOTM.
- **Auto recovery triggering too early/late**: Graph `Drive/Auto/PathFollowingErrorM` and overlay `Drive/Auto/IsOffPath`. If error gradually climbs, path constraints may be too aggressive.
- **Auto recovery flickering**: If `IsOffPath` rapidly toggles, the robot is hovering around the 0.15m threshold - may need hysteresis.
- **Shooter not at speed**: Compare `Shooter/TargetRPM` vs actual RPM values. Large gap = spin-up too slow or mechanical issue.
- **Robot not rotating to target**: Check `Drive/Aim/CurrentHeadingDeg` vs `TargetAngleDeg`. If they match but `ErrorDegHub` is nonzero, the 180-degree offset may be wrong.
- **SOTM lead looks wrong**: Compare `DynamicHubPose` vs `StaticHubPose` on the field view. The offset should point opposite to robot velocity direction.
