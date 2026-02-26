package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Configs;

public class Climber extends SubsystemBase {
    private double ClimberExtPos = ClimberConstants.CLIMBER_EXTENDED_POS;
    private double ClimberRetPos = ClimberConstants.CLIMBER_RETRACTED_POS;

    private SparkFlex ClimbMotorLeft = new SparkFlex(ClimberConstants.CLIMBER_LEFT_ID, MotorType.kBrushless);

    private SparkClosedLoopController climbLeftController = ClimbMotorLeft.getClosedLoopController();

    private RelativeEncoder ClimberEncoder = ClimbMotorLeft.getEncoder();

    public Climber() {
        ClimbMotorLeft.configure(Configs.ClimberSubsystem.ClimbMotorLeftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        ClimberEncoder.setPosition(0);
    }

    public void Climb() {
        climbLeftController.setSetpoint(ClimberExtPos, ControlType.kMAXMotionPositionControl);
    }

    public void ClimbDown() {
        climbLeftController.setSetpoint(ClimberRetPos, ControlType.kMAXMotionPositionControl);
    }

    public void stopClimber() {
        ClimbMotorLeft.set(0);
        // ClimbMotorRight.set(0);
    }

    public Command runClimbCommand() {
        return new RunCommand(() -> Climb(), this)
                .finallyDo(interrupted -> stopClimber());
    }

    public Command runClimberDownCommand() {
        return new RunCommand(() -> ClimbDown(), this)
                .finallyDo(interrupted -> stopClimber());
    }

    @Override
    public void periodic() {
    }
}
