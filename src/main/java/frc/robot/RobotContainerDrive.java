// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import swervelib.SwerveInputStream;

/**
 * Minimal container for drivebase-only testing.
 * Avoids constructing other subsystems so missing CAN devices won't fault.
 */
public class RobotContainerDrive {

  private final CommandXboxController driverXbox = new CommandXboxController(0);

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));


   private final SendableChooser<Command> autoChooser;
 private LoggedDashboardChooser<Command> loggedAutoChooser;
  private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX()*-1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(1.2)
      .scaleRotation(1.2)
      .allianceRelativeControl(true)
      // .aim(Constants.DrivebaseConstants.getHubPose().toPose2d())
      // .aimWhile(driverXbox.b())
      ;

   private final SwerveInputStream driveSlowerAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX()*-1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.4)
      .scaleRotation(0.4)
      .allianceRelativeControl(true)
      ;

  
  public RobotContainerDrive() {
 

    SmartDashboard.putNumber("Heading Bias Deg", 0.0);
    // Tunable gain: radians of bias -> radians/sec of angular velocity
    SmartDashboard.putNumber("Heading Bias Gain", 0.0);
    configureBindings();
    setDefaultDriveCommand();

      autoChooser = AutoBuilder.buildAutoChooser();


   // Set the default auto (do nothing)
   autoChooser.setDefaultOption("Do Nothing", Commands.none());


   // // Add a simple auto option to have the robot drive forward for 1 second then
   // // stop
   // autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));


   // Put the autoChooser on the SmartDashboard
   SmartDashboard.putData("Auto Chooser", autoChooser);


   loggedAutoChooser = new LoggedDashboardChooser<>("Auto Routine", autoChooser);
  }

  private void configureBindings() {

       Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(
       () -> applyHeadingBias(driveAngularVelocity.get()));

    Command driveSlowerFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(
       () -> applyHeadingBias(driveSlowerAngularVelocity.get()));
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.rightTrigger(0.5).whileTrue(driveSlowerFieldOrientedAnglularVelocity).toggleOnFalse(driveFieldOrientedAnglularVelocity);
  }

  private void setDefaultDriveCommand() {
    if (Constants.USE_ROBOT_RELATIVE) {
      drivebase.setDefaultCommand(
          drivebase.run(() -> drivebase.drive(driveAngularVelocity.get())));
    } else {
      Supplier<ChassisSpeeds> fieldOriented = () -> applyHeadingBias(driveAngularVelocity.get());
      drivebase.setDefaultCommand(drivebase.driveFieldOriented(fieldOriented));
    }
  }


  public Command getAutonomousCommand() {
     return loggedAutoChooser.get();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void logControllerInputs() {
    Logger.recordOutput("Input/Driver/LeftX", driverXbox.getLeftX());
    Logger.recordOutput("Input/Driver/LeftY", driverXbox.getLeftY());
    Logger.recordOutput("Input/Driver/RightX", driverXbox.getRightX());
    Logger.recordOutput("Input/Driver/RightY", driverXbox.getRightY());
    Logger.recordOutput("Input/Driver/LeftTrigger", driverXbox.getLeftTriggerAxis());
    Logger.recordOutput("Input/Driver/RightTrigger", driverXbox.getRightTriggerAxis());
  }

  private ChassisSpeeds applyHeadingBias(ChassisSpeeds speeds) {
    // Toggle to enable heading bias; false means pass-through.
    boolean headingBiasEnabled = SmartDashboard.getBoolean("headingBiasEnabled", false);
    if (!headingBiasEnabled) {
      return speeds;
    }
    // Requested heading bias in degrees; 0 means disabled.
    double biasDeg = SmartDashboard.getNumber("Heading Bias Deg", 0.0);
    // Gain mapping bias radians -> added omega (rad/sec).
    double gain = SmartDashboard.getNumber("Heading Bias Gain", 0.0);

    // Default to normal driving (no bias).
    double omega = speeds.omegaRadiansPerSecond;
    if (biasDeg != 0.0 && gain != 0.0) {
      // Convert degrees to radians, then scale into an omega offset.
      double biasRad = Math.toRadians(biasDeg);
      double additionalOmega = gain * biasRad;
      // Leave vx/vy alone; only add a small angular velocity component.
      omega += additionalOmega;
    }

    return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);
  }

}
