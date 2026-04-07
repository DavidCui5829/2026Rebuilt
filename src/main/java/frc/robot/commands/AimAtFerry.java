package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import swervelib.SwerveInputStream;


public class AimAtFerry extends Command
{

  public final SwerveSubsystem   swerveSubsystem;
  public final SwerveInputStream swerveInputStream;

  public AimAtFerry(SwerveSubsystem swerveSubsystem, SwerveInputStream swerveInputStream)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveInputStream = swerveInputStream.copy();
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize()
  {
    swerveInputStream.aimHeadingOffset(Rotation2d.fromDegrees(180))
                     .aimHeadingOffset(true)
                     .aimWhile(true);
  }

  @Override
  public void execute()
  {
    // shoot at the nearest ferry pos
    var ferry = swerveSubsystem.getDynamicFerryLocation();

    swerveInputStream.aim(() -> ferry);

    swerveSubsystem.getField()
                   .getObject("AimTarget")
                   .setPose(ferry);

    swerveSubsystem.driveFieldOriented(swerveInputStream.get());
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    swerveInputStream.aimWhile(false);
    swerveSubsystem.getField().getObject("AimTarget").setPoses(List.of());
  }
}