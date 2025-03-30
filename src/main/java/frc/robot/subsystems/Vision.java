package frc.robot.subsystems;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logger;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
  RobotContainer container;
  PhotonCamera photonCamera;
  PhotonPoseEstimator photonPoseEstimator;

  // Returns IO exception, should handle this
  String aprilTagLayoutPath = "2025-reefscape-andymark.json";
  AprilTagFieldLayout fieldLayout;

  {
    try {
      fieldLayout = new AprilTagFieldLayout(aprilTagLayoutPath);
      Logger.Log("Field layout loaded");
    } catch (IOException e) {
      e.printStackTrace();
      fieldLayout = null; // Handle the error gracefully
    }
  }

  // AprilTagFields fields = AprilTagFields.k2025ReefscapeAndyMark;
  // AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(fields);

  Transform3d robotToCam =
      new Transform3d(
          new Translation3d(0.2, 0.0, 0.3), // adjust for your camera's position on the robot
          new Rotation3d(0, 0, 0));

  public Vision() {
    container = new RobotContainer();
    photonCamera = new PhotonCamera("Arducam_OV2311_USB_Camera (1)"); // ("PhotonVision");
    Logger.Log("PhotonCamera loaded: " + photonCamera);

    photonPoseEstimator =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
    Logger.Log("PhotonPoseEstimator loaded: " + photonPoseEstimator);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    List<PhotonPipelineResult> result = photonCamera.getAllUnreadResults();
    PhotonPipelineResult lastResult = result.get(result.size() - 1);
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update(lastResult);
  }

  @Override
  public void periodic() {
  
    
  
    fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    List<PhotonPipelineResult> result = photonCamera.getAllUnreadResults();
    PhotonPipelineResult lastResult = result.get(result.size() - 1);
    if (lastResult.hasTargets()) {
      Logger.LogPeriodic("YES vision targets found");
      photonPoseEstimator.setReferencePose(container.swerve.getEstimatedPose());
      Optional<EstimatedRobotPose> optionalEstimatedPose = photonPoseEstimator.update(lastResult);

      optionalEstimatedPose.ifPresent(
          estimatedRobotPose -> {
            Pose2d pose2d = estimatedRobotPose.estimatedPose.toPose2d();
            Logger.LogPeriodic("Got vision pose: " + pose2d);
            SmartDashboard.putNumber("Estimated X", pose2d.getX());
            SmartDashboard.putNumber("Estimated Y", pose2d.getY());
            SmartDashboard.putNumber("Estimated Heading", pose2d.getRotation().getDegrees());

            // To update to the pose we calculate with this
            // container.swerve.resetOdometry(pose2d);
          });
          SmartDashboard.putNumber("End of vision loop", 1);
        }  else {
      Logger.LogPeriodic("NO vision targets found");           
    }
  }
}
