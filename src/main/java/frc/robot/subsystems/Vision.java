package frc.robot.subsystems;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Logger;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  KrakenSwerve swerve;

  PhotonCamera photonCamera;
  PhotonPoseEstimator photonPoseEstimator;

  String aprilTagLayoutPath = "2025-reefscape-andymark.json";

  AprilTagFieldLayout fieldLayout;

  // cache of transform to closest April tag    
  double xOffset;
  double yOffset;
  double angleOffset;



  public Vision(KrakenSwerve swerve) {
    this.swerve = swerve;
    photonCamera = new PhotonCamera("Arducam_OV2311_USB_Camera"); // ("PhotonVision");
    Logger.Log("PhotonCamera loaded: " + photonCamera);

    try {
      Path path = Filesystem.getDeployDirectory().toPath().resolve(aprilTagLayoutPath);
      Logger.Log("Got path: " + path.toAbsolutePath());
      fieldLayout = new AprilTagFieldLayout(path.toAbsolutePath());
      Logger.Log("Field layout loaded");
      Logger.Log("April Tag 1 in layout: " + fieldLayout.getTagPose(1).isPresent());

      Transform3d robotToCam =
          new Transform3d(
              new Translation3d(0.0, 0.2, 0.3), // adjust for your camera's position on the robot
              new Rotation3d(0, 0, 0));

      photonPoseEstimator =
          new PhotonPoseEstimator(
              fieldLayout,
              PoseStrategy.CONSTRAINED_SOLVEPNP,
              robotToCam); // PoseStrategy.CLOSEST_TO_REFERENCE_POSE
      Logger.Log("PhotonPoseEstimator loaded: " + photonPoseEstimator);
    } catch (Exception e) {
      Logger.Log("Error loading field layout: " + e.getMessage());
    }
  }

  public boolean canScore(Boolean isLeft) {
 
    // Parameters - you can tune these
    double distanceThreshold = 2.55; // meters
    double distanceMinimum = 2.2; // meters
    double yThresholdLeft = -0.85; // meters
    double yThresholdLeftMax = -1.0; // meters
    double yThresholdRight = -0.7; // meters
    double yThresholdRightMax = -0.6; // meters
    double angleThreshold = 2.0; // degrees

    boolean canScoreLeft =
        xOffset < distanceThreshold
            && xOffset > distanceMinimum
            && yOffset < yThresholdLeft
            && yOffset > yThresholdLeftMax
            && angleOffset < angleThreshold
            && angleOffset > -4.0;
    boolean canScoreRight =
        xOffset < distanceThreshold
            && xOffset > distanceMinimum
            && yOffset > yThresholdRight
            && yOffset < yThresholdRightMax
            && angleOffset < angleThreshold
            && angleOffset > -4.0;

    if (isLeft) {
      return canScoreLeft;
    } else {
      return canScoreRight;
    }
  }

  @Override
  public void periodic() {

    try {
      List<PhotonPipelineResult> result = photonCamera.getAllUnreadResults();
      if (result.size()==0) {
        return;
      }
      PhotonPipelineResult lastResult = result.get(result.size() - 1);
      if (!lastResult.hasTargets()) {
        return;
      }
  
      PhotonTrackedTarget target = lastResult.getBestTarget();
      int tagId = target.getFiducialId();
  
      List<Integer> noScoreTags = List.of(1, 2, 3, 4, 5, 12, 13, 14, 15, 16);
  
      if (noScoreTags.contains(tagId)) {
        return;
      }
  
      Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(tagId);
      if (tagPoseOptional.isEmpty()) {
        return;
      }
  

      // Optional<EstimatedRobotPose> poseEstimate = photonPoseEstimator.update(lastResult);
      // if (poseEstimate.isEmpty()) {
      //   return;
      // }
  
      // Pose2d robotPose = poseEstimate.get().estimatedPose.toPose2d();
      // Pose2d tagPose = tagPoseOptional.get().toPose2d();
  
      // Transform2d robotToTag = new Transform2d(robotPose, tagPose);

      Transform3d robotToTag = target.getBestCameraToTarget();
      Logger.LogPeriodic("robotToTag: " + robotToTag);
  
      xOffset = robotToTag.getX();
      yOffset = robotToTag.getY();
      angleOffset = robotToTag.getRotation().getZ();
    // Optional telemetry
    SmartDashboard.putNumber("Score X", xOffset);
    SmartDashboard.putNumber("Score Y", yOffset);
    SmartDashboard.putNumber("Score Angle", angleOffset);
    SmartDashboard.putBoolean("Can Score Left L2", canScore(true));
    SmartDashboard.putBoolean("Can Score Right L2", canScore(false));

    } catch (Exception e) {
      Logger.Log("Error: " + e);
    }  
  }
}
