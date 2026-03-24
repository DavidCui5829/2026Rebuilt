package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.REVLibError;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.FunnelConstants;
import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {

    // AdvantageKit logging
    private double desiredPercent = 0.0;

    private SparkFlex FunnelMotor = new SparkFlex(FunnelConstants.FUNNEL_ID, MotorType.kBrushless);
    private SparkClosedLoopController funnelController = FunnelMotor.getClosedLoopController();
  

    public Funnel() {
        FunnelMotor.configure(Configs.FunnelSubsystem.FunnelMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void runFunnelBack() {
        funnelController.setSetpoint(FunnelConstants.REVERSE_FUNNEL_RPM, ControlType.kMAXMotionVelocityControl);

    }

    public void runFunnel() {
        funnelController.setSetpoint(FunnelConstants.FUNNEL_RPM, ControlType.kMAXMotionVelocityControl);
    }

    public void stopFunnel() {
        desiredPercent = 0.0;
        FunnelMotor.set(0);
        // IntakeRightMotor.set(0);
    }

    public Command runFunnelCommand() {
        return new RunCommand(() -> runFunnel(), this)
                .finallyDo(interrupted -> stopFunnel());
    }

    public Command runReverseFunnelCommand() {
        return new RunCommand(() -> runFunnelBack(), this)
                .finallyDo(interrupted -> stopFunnel());
    }

    public Command stopFunnCommand() {
        return new RunCommand(() -> stopFunnel(), this);
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        Logger.recordOutput("Funnel/DesiredPercent", desiredPercent);
        // Applied voltage to intake motor.
        Logger.recordOutput("Funnel/AppliedVolts", FunnelMotor.getAppliedOutput() * FunnelMotor.getBusVoltage());
    }
}