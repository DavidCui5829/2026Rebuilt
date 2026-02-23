package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Configs;

public class Climber extends SubsystemBase {

    private SparkFlex ClimbMotorLeft = new SparkFlex(ClimberConstants.CLIMBER_LEFT_ID, MotorType.kBrushless);
    // private SparkFlex ClimbMotorRight = new
    // SparkFlex(ClimberConstants.CLIMBER_RIGHT_ID, MotorType.kBrushless);

    private SparkClosedLoopController climbLeftController = ClimbMotorLeft.getClosedLoopController();
    // private SparkClosedLoopController climbRightController =
    // ClimbMotorRight.getClosedLoopController();

    public Climber() {
        ClimbMotorLeft.configure(Configs.ClimberSubsystem.ClimbMotorLeftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        // ClimbMotorRight.configure(Configs.ClimberSubsystem.ClimbMotorRightConfig,
        // ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void Climb() {
        ClimbMotorLeft.set(ClimberConstants.CLIMBER_SPEED);
        // ClimbMotorRight.set(ClimberConstants.CLIMBER_SPEED);
    }

    public void ClimbDown() {
        ClimbMotorLeft.set(ClimberConstants.CLIMBER_DOWN_SPEED);
        // ClimbMotorRight.set(ClimberConstants.CLIMBER_DOWN_SPEED);
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
