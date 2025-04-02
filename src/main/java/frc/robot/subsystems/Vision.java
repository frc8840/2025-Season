package frc.robot.subsystems;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
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

  @Override
  public void periodic() {

    double timeSeconds = System.currentTimeMillis() / 1000.0;
    photonPoseEstimator.addHeadingData(timeSeconds, swerve.getYaw());

    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    if (results.isEmpty()) {
      // Logger.LogPeriodic("No results");
      return;
    }
    PhotonPipelineResult lastResult = results.get(results.size() - 1);
    if (!lastResult.hasTargets()) {
      Logger.LogPeriodic("NO vision targets found");
      return;
    }
    PhotonTrackedTarget target = lastResult.getBestTarget();
    // Logger.LogPeriodic("Best vision target: " + target.toString());
    Logger.LogPeriodic("Target ID: " + target.getFiducialId());
    Logger.LogPeriodic("yaw: " + target.getYaw());
    Logger.LogPeriodic("pitch: " + target.getPitch());
    Logger.LogPeriodic("area: " + target.getArea());
    // Logger.LogPeriodic("Tag Pose from layout: " + fieldLayout.getTagPose(target.getFiducialId()));
    Logger.LogPeriodic("Best Transform: " + target.getBestCameraToTarget());
    Logger.LogPeriodic("Alt  Transform: " + target.getAlternateCameraToTarget());

    // photonPoseEstimator.setReferencePose(swerve.getEstimatedPose()); // only needed if we're
    // using PoseStrategy.CLOSEST_TO_REFERENCE_POSE
    Logger.LogPeriodic("Number of results: " + results.size());
    // Logger.LogPeriodic("Result Info: " + results.get(0));
    Optional<EstimatedRobotPose> optionalEstimatedPose = photonPoseEstimator.update(lastResult);
    // Logger.LogPeriodic("OptionalEstimatedPose is empty? " + optionalEstimatedPose.isEmpty());

    optionalEstimatedPose.ifPresent(
        estimatedRobotPose -> {
          Pose2d pose2d = estimatedRobotPose.estimatedPose.toPose2d();
          Logger.LogPeriodic("Got vision pose: " + pose2d);
          // Logger.LogPeriodic(
          //     "Estimated X: "
          //         + pose2d.getX()
          //         + " Estimated Y: "
          //         + pose2d.getY()
          //         + " Estimated Heading: "
          //         + pose2d.getRotation().getDegrees());
          SmartDashboard.putNumber("Estimated X", pose2d.getX());
          SmartDashboard.putNumber("Estimated Y", pose2d.getY());
          SmartDashboard.putNumber("Estimated Heading", pose2d.getRotation().getDegrees());
          // To update to the pose we calculate with this
          // swerve.resetOdometry(pose2d);
        });
    SmartDashboard.putNumber("End of vision loop", 1);
  }
}
