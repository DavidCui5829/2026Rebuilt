package frc.robot.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetection extends SubsystemBase {
    
    // Name of our camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    // Query the latest result from PhotonVision
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    // Check if the latest result has any targets.
    boolean hasTargets = (results.size() > 0);

    public double getAlignToFuelRotation() {
        // Simple P-controller to compute the rotation command to align the robot to the target.
        // Tweak kP and maxRot for your robot.
        final double kP = 0.02;    // proportional gain (tune)
        final double maxRot = 0.5; // max rotational speed (tune)

        if (hasTargets) {
            var bestTarget = results.get(0).getBestTarget();
            // PhotonTrackedTarget#getYaw() returns the horizontal offset (degrees) from camera to target.
            double yawDegrees = bestTarget.getYaw();
            double rotationCmd = yawDegrees * kP;
            // clamp
            rotationCmd = Math.max(-maxRot, Math.min(maxRot, rotationCmd));
            return rotationCmd;
        } else {
            // No target: no rotation
            return 0.0;
        }
    }

    @Override
    public void periodic() {
        results = camera.getAllUnreadResults();
        hasTargets = (results.size() > 0);
    }

    @Override
    public void simulationPeriodic() {
        results = camera.getAllUnreadResults();
        hasTargets = (results.size() > 0);

        Logger.recordOutput("ObjectDetection/PipelineHasTargets", hasTargets);
        
        results.forEach(result -> {
            Logger.recordOutput("ObjectDetection/TargetPose", result.getBestTarget().getBestCameraToTarget());
        });

        if (hasTargets) {
            var bestTarget = results.get(0).getBestTarget();
            var targetPose = bestTarget.getBestCameraToTarget();
            Logger.recordOutput("ObjectDetection/AlignToFuelPose", targetPose);
        }


    }

}
