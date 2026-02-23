package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.PushoutConstants;
import frc.robot.Configs;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Pushout extends SubsystemBase {

    // AdvantageKit logging
    private double desiredPercent = 0.0;

    private SparkFlex PushoutMotor = new SparkFlex(PushoutConstants.PUSHOUT_ID, MotorType.kBrushless);
    private SparkClosedLoopController PushoutController = PushoutMotor.getClosedLoopController();
    // private SparkFlex PushoutRightMotor = new
    // SparkFlex(PushoutConstants.PUSHOUT_RIGHT_ID, MotorType.kBrushless);
    // private SparkClosedLoopController PushoutRightController =
    // PushoutRightMotor.getClosedLoopController();

    private RelativeEncoder pushoutEncoder = PushoutMotor.getEncoder();
    // private final RelativeEncoder pushoutRightEncoder =
    // PushoutRightMotor.getEncoder();

    // private double PushoutRightExtended = PushoutConstants.PUSHOUT_EXTENDED_POS;
    private double PushoutExtended = PushoutConstants.PUSHOUT_EXTENDED_POS;
    // private double PushoutRightRetracted =
    // PushoutConstants.PUSHOUT_RETRACTED_POS;
    private double PushoutRetracted = PushoutConstants.PUSHOUT_RETRACTED_POS;
    // private double PushoutRightExtendedAgitate =
    // PushoutConstants.PUSHOUT_EXTENDED_AGITATE_POS;
    private double PushoutExtendedAgitate = PushoutConstants.PUSHOUT_EXTENDED_AGITATE_POS;
    // private double PushoutRightRetractedAgitate =
    // PushoutConstants.PUSHOUT_RETRACTED_AGITATE_POS;
    private double PushoutRetractedAgitate = PushoutConstants.PUSHOUT_RETRACTED_AGITATE_POS;

    public Pushout() {
        PushoutMotor.configure(Configs.PushoutSubsystem.PushoutMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        // PushoutRightMotor.configure(Configs.PushoutSubsystem.PushoutRightMotorConfig,
        // ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);
        pushoutEncoder.setPosition(0);
        // pushoutRightEncoder.setPosition(0);
    }

    public void PushIntake() {
        PushoutController.setSetpoint(PushoutExtended, ControlType.kMAXMotionPositionControl);
        // PushoutRightController.setSetpoint(PushoutRightExtended,
        // ControlType.kMAXMotionPositionControl);

    }

    public void RetractIntake() {
        PushoutController.setSetpoint(PushoutRetracted, ControlType.kMAXMotionPositionControl);
        // PushoutRightController.setSetpoint( PushoutRightRetracted,
        // ControlType.kMAXMotionPositionControl);
    }

    public void SmallPush() {
        // double leftNow = pushoutLeftEncoder.getPosition();
        // double rightNow = pushoutRightEncoder.getPosition();
        PushoutController.setSetpoint(PushoutExtendedAgitate, ControlType.kMAXMotionPositionControl);
        // PushoutRightController.setSetpoint(PushoutRightExtendedAgitate,
        // ControlType.kMAXMotionPositionControl);
    }

    public void SmallRetract() {
        // double leftNow = pushoutLeftEncoder.getPosition();
        // double rightNow = pushoutRightEncoder.getPosition();
        PushoutController.setSetpoint(PushoutRetractedAgitate, ControlType.kMAXMotionPositionControl);
        // PushoutRightController.setSetpoint(PushoutRightRetractedAgitate,
        // ControlType.kMAXMotionPositionControl);
    }

    public void PushoutDutyCycle() {
        PushoutMotor.set(0.3);
    }

    public void StopPushout() {
        PushoutMotor.set(0);
    }

    // public void StopPushing() {
    // // PushoutLeftController.setSetpoint(0,
    // ControlType.kMAXMotionPositionControl);
    // PushoutRightController.setSetpoint(0, ControlType.kMAXMotionPositionControl);
    // }

    public void Agitate() {
        SmallPush();
        Timer.delay(PushoutConstants.PUSHOUT_AGITATE_WAIT);
        SmallRetract();
        Timer.delay(PushoutConstants.PUSHOUT_AGITATE_WAIT);

    }

    public Command PushCommand() {
        return new RunCommand(() -> PushoutDutyCycle(), this)
                .finallyDo(interrupted -> StopPushout());
    }

    public Command RetractCommand() {
        return new RunCommand(() -> RetractIntake(), this)
                .finallyDo(interrupted -> RetractIntake());
    }

    public Command AgitateCommand() {
        return new RunCommand(() -> Agitate(), this)
                .finallyDo(interrupted -> RetractIntake());
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        Logger.recordOutput("Pushout/DesiredPercent", desiredPercent);
        // Applied voltage to intake motor.
        // Logger.recordOutput("Pushout/AppliedVolts",
        // PushoutLeftMotor.getAppliedOutput() * PushoutLeftMotor.getBusVoltage());
    }
}
