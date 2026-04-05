package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants.Hub;
import java.util.List;
import swervelib.SwerveInputStream;


public class AutoAimCommand extends Command
{

  private final SwerveSubsystem   swerveSubsystem;
  private final SwerveInputStream swerveInputStream;
  private Pose2d targetPose;

  public AutoAimCommand(SwerveSubsystem swerveSubsystem, SwerveInputStream swerveInputStream)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveInputStream = swerveInputStream.copy();
//    this.swerveInputStream.scaleTranslation(slowScale);
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize()
  {
    targetPose = AllianceFlipUtil.apply(new Pose2d(Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero));
    swerveSubsystem.getField().getObject("AimTarget").setPose(targetPose);

    swerveInputStream.aim(targetPose)
                     .aimHeadingOffset(Rotation2d.fromDegrees(180))
                     .aimHeadingOffset(true)
                     .aimWhile(true);
    /*

        .aim(() -> drivebase.getDynamicHubLocation())
        // .aim(() -> isInAllianceZone() ? drivebase.getDynamicHubLocation() :
        // drivebase.getDynamicFerryLocation())
        // .aimLock(Angle.ofBaseUnits(1, Degrees))
        .aimWhile(dc().rightTrigger())
        // .aimWhile(driverXbox.leftTrigger())
        .aimLookahead(Time.ofBaseUnits(0.2, Seconds))
        .aimFeedforward(0.0001, 0.0001, 0.00018)
        .aimHeadingOffset(Rotation2d.fromDegrees(183))
        .aimHeadingOffset(true)
     */

  }

  @Override
  public void execute()
  {
    swerveSubsystem.driveFieldOriented(swerveInputStream.get());
  }

  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    swerveInputStream.aimWhile(false);
    swerveSubsystem.getField().getObject("AimTarget").setPoses(List.of());
  }
}
