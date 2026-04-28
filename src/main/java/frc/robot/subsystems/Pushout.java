package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

import edu.wpi.first.math.filter.Debouncer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.PushoutConstants;
import frc.robot.Configs;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Pushout extends SubsystemBase {

    // AdvantageKit logging
    private double desiredPercent = 0.0;

    private SparkFlex PushoutMotorL = new SparkFlex(PushoutConstants.PUSHOUT_ID_L, MotorType.kBrushless);
    private SparkClosedLoopController PushoutControllerL = PushoutMotorL.getClosedLoopController();
    private SparkFlex PushoutMotorR = new SparkFlex(PushoutConstants.PUSHOUT_ID_R, MotorType.kBrushless);
    private SparkClosedLoopController PushoutController2 = PushoutMotorR.getClosedLoopController();


    private final Angle hardLowerLimit = Degrees.of(0);

    // private SparkFlex PushoutRightMotor = new
    // SparkFlex(PushoutConstants.PUSHOUT_RIGHT_ID, MotorType.kBrushless);
    // private SparkClosedLoopController PushoutRightController =
    // PushoutRightMotor.getClosedLoopController();

    private RelativeEncoder pushoutEncoder = PushoutMotorL.getEncoder();

    double minVelocity = -350.0;
    // private final RelativeEncoder pushoutRightEncoder =
    // PushoutRightMotor.getEncoder();

    public Pushout() {
        PushoutMotorL.configure(Configs.PushoutSubsystem.PushoutMotorConfigL, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        PushoutMotorR.configure(Configs.PushoutSubsystem.PushoutMotorConfigR, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
           
        pushoutEncoder.setPosition(0);
        
    }

    public void PushIntake() {
        PushoutControllerL.setSetpoint(PushoutConstants.PUSHOUT_EXTENDED_POS, ControlType.kMAXMotionPositionControl);
    }

    public void RetractIntake() {
        PushoutControllerL.setSetpoint(PushoutConstants.PUSHOUT_RETRACTED_POS, ControlType.kMAXMotionPositionControl);
    }

    public void FullyRetract() {
        PushoutControllerL.setSetpoint(PushoutConstants.FULLY_RETRACTED_POS, ControlType.kMAXMotionPositionControl);
    }

    public void ResetEncoder() {
        pushoutEncoder.setPosition(0);
    }

    public void StopPushout() {
        PushoutMotorL.set(0);
    }

    public void PushoutDutycyle() {
        PushoutMotorL.set(0.8);
    }

    public void PushoutDutycyleRetract() {
        PushoutMotorL.set(-0.8);
    }
    public void CheeksyAgitation() {

        PushoutControllerL.setSetpoint(minVelocity, ControlType.kMAXMotionVelocityControl);
    }

    public Command CheeksyAgitationCommand() {
        return this.run(() -> CheeksyAgitation())
                .finallyDo(() -> {
                    StopPushout();
                });
    }
    public Command HomingCommand(double threshold) {
        Debouncer currentDebouncer = new Debouncer(0.2);

        return new RunCommand(() -> PushoutControllerL.setSetpoint(minVelocity, ControlType.kMAXMotionVelocityControl),
                this)
                .until(() -> (PushoutMotorL.getEncoder().getVelocity() >= threshold))
                .finallyDo(() -> {
                    StopPushout();
                });
    }

    public Command PushoutDutycyleCommand() {
        return this.run(() -> PushoutDutycyle())
                .finallyDo(interrupted -> StopPushout());
    }

    public Command PushoutDutycyleRetractCommand() {
        return this.run(() -> PushoutDutycyleRetract())
                .finallyDo(interrupted -> StopPushout());

    }

    public Command PushCommand() {
        return this.run(() -> PushIntake())
                .finallyDo(interrupted -> StopPushout());

    }

    public Command ResetEncoderCommand() {
        return this.runOnce(() -> ResetEncoder());
    }

    public Command RetractCommand() {
        return this.runOnce(() -> RetractIntake());

    }

    public Command FullyRetractCommand() {
        return this.runOnce(() -> FullyRetract());
    }

    public Command AgitateCommand() {
        final double[] pullPositions = { 12.5, 10, 7, 5, 3 }; // each time it pushes less far in
        final double[] pushPositions = { 15, 13.5, 10, 8.5, 6 }; // each time it pulls further out
        final double finalPos = 4; // pull to this position and idle there after agitation done
        final double waitTime = PushoutConstants.PUSHOUT_AGITATE_WAIT;
        final double waitBetween = PushoutConstants.PUSHOUT_BETWEEN;
        Command agitate = Commands.sequence(
                // push to 11 & pull to 8
                runOnce(() -> PushoutControllerL.setSetpoint(pullPositions[0], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutControllerL.setSetpoint(pushPositions[0], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(waitBetween),

                // push to 9 & pull to 6
                runOnce(() -> PushoutControllerL.setSetpoint(pullPositions[1], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutControllerL.setSetpoint(pushPositions[1], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(waitBetween),

                // push to 7 & pull to 4
                runOnce(() -> PushoutControllerL.setSetpoint(pullPositions[2], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutControllerL.setSetpoint(pushPositions[2], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(waitBetween),

                runOnce(() -> PushoutControllerL.setSetpoint(pullPositions[3], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutControllerL.setSetpoint(pushPositions[3], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(waitBetween),

                runOnce(() -> PushoutControllerL.setSetpoint(pullPositions[4], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutControllerL.setSetpoint(pushPositions[4], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(waitBetween),

                // end pos
                runOnce(() -> PushoutControllerL.setSetpoint(finalPos, ControlType.kMAXMotionPositionControl)),
                Commands.idle(this)

        ).finallyDo(interrupted -> PushIntake());
        agitate.addRequirements(this);
        return agitate;
    }

    public Command runDefaultCommand() {
        return new RunCommand(() -> StopPushout(), this);
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
