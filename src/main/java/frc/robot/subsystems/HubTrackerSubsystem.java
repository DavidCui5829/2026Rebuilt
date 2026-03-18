package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubTrackerSubsystem extends SubsystemBase
{

    private Field2d field = new Field2d();
    private final SwerveSubsystem drivebase;
    private FieldObject2d circle;

    private final Pose2d hubPose;
    private double radius = 1; // radius to represent time left (circle gets smaller when shift ending)

    boolean show = false;
    int x = 0;
    
    public HubTrackerSubsystem(SwerveSubsystem drivebase)
    {
        this.drivebase = drivebase;
        circle = field.getObject("Circle"); 
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        hubPose = switch (alliance)
        {
            case Blue -> new Pose2d(1.9, 4.1, new Rotation2d());
            case Red -> new Pose2d(16.54 - 1.9, 4.1, new Rotation2d());
            default -> new Pose2d(1.9, 4.1, new Rotation2d());
        };
        SmartDashboard.putData("Field", field);
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) return false;

        if (DriverStation.isAutonomousEnabled()) return true; // Always enabled in auton

        if (!DriverStation.isTeleopEnabled()) return false; // If we're disabled then were probably not playing

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) return true;

        boolean isRedActiveFirst;
        switch (gameData.charAt(0)) {
        case 'R':
            isRedActiveFirst = false;
            break;
        case 'B':
            isRedActiveFirst = true;
            break;
        default:
            return false;
        }

        boolean shiftOneActive = switch (alliance.get()) {
        case Red -> isRedActiveFirst;
        case Blue -> !isRedActiveFirst;
        };

        if (matchTime >= 130) // transition shift, always active
        {
            radius = (140 - matchTime) / 10;
            return true;
        } 
        else if (matchTime >= 105) { // shift 1
            radius = (130 - matchTime) / 15;
            return shiftOneActive;
        } 
        else if (matchTime >= 80) // shift 2
        {
            radius = (105 - matchTime) / 15;
            return !shiftOneActive;
        } 
        else if (matchTime >= 55) // shift 3
        {
            radius = (80 - matchTime) / 15;
            return shiftOneActive;
        } 
        else if (matchTime >= 30) // shift 4
        {
            radius = (55 - matchTime) / 15;
            return !shiftOneActive;
        } 
        else
        {
            radius = matchTime / 30;
            return true; // Endgame, always active
        }
  }

//   public Command isHubActiveCommand()
//   {
//     return run(()->
//     {
//         isHubActive();
//     });
//   }

  public List<Pose2d> createCircle(Pose2d center, double r, int pts)
  {
    List<Pose2d> circle = new ArrayList<>();
    for(int i=0;i<pts;++i)
    {
        double angle = (i * (360.0/(double)pts)) * Math.PI / 180;
        
        double xOffset = r * Math.cos(angle);
        double yOffset = r * Math.sin(angle);

        circle.add(new Pose2d(center.getX() + xOffset, center.getY() + yOffset, new Rotation2d()));
    }

    return circle;
  }

  public void runPeriodic()
  {
    field.setRobotPose(drivebase.getPose());
    x = (x == 1) ? 0 : 1;
    // Ok so this is a really weird (but genius) line of code i came up with (yk it aint AI because AI wouldn't make something like this lol)
    // essentially, the circle will be shown either when it is an INACTIVE hub, or every other frame of active shift
    // this way, if it is active, it blinks. this is my workaround for not being able to change color of circle
    if(!isHubActive() || x == 0) circle.setPoses(createCircle(hubPose, radius, 20));
    else circle.setPoses(); // clears circle when not showing
  }

  @Override
  public void periodic()
  {
    runPeriodic();
  }

  @Override
  public void simulationPeriodic()
  {
    runPeriodic();
  }

}