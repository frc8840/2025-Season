package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logger;

public class SparkSwerve extends SubsystemBase {

  private final Pigeon2 gyro;

  private SwerveDriveOdometry odometer;
  private SparkSwerveModule[] mSwerveMods;

  private Field2d field;

  public SparkSwerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    zeroGyro();

    SwerveModulePosition[] startPositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
        };
    odometer =
        new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics, new Rotation2d(0), startPositions);

    // order is always FRONT LEFT, FRONT RIGHT, BACK LEFT, BACK RIGHT
    mSwerveMods =
        new SparkSwerveModule[] {
          new SparkSwerveModule(0, Constants.Swerve.FLconstants),
          new SparkSwerveModule(1, Constants.Swerve.FRconstants),
          new SparkSwerveModule(2, Constants.Swerve.BLconstants),
          new SparkSwerveModule(3, Constants.Swerve.BRconstants)
        };

    field = new Field2d();
    // SmartDashboard.putData("Field", field);
    // Logger.Log("mSwerveMods.length=" + mSwerveMods.length);

    // Configure AutoBuilder last
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    // starting to use pathplanner
    Logger.Log("about to configureHolonomic");
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveFromSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds. Also optionally outputs individual module
        // feedforwards
        new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(0.01, 0.0, 0.0), // Translation PID constants
            new PIDConstants(0.01, 0.0, 0.0) // Rotation PID constants
            ),
        config, // The robot configuration
        () -> {

          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
    Logger.Log("completed configureHolonomic");
  }

  // translation and rotation are the desired behavior of the robot at this moment
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    // first, we compute our desired chassis speeds
    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    driveFromSpeeds(chassisSpeeds);
  }

  // used by tele
  public void driveFromSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
    // do we need the below?
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
    // Constants.Swerve.maxSpeed);
    setModuleStates(swerveModuleStates);
  }

  public void runEachMotorForOneSecond() {
    for (SparkSwerveModule module : mSwerveMods) {
      // Run the current module's motor at full speed (or desired speed).
      module.setDesiredState(
          new SwerveModuleState(Constants.Swerve.maxSpeedMetersPerSecond, new Rotation2d()));
      Logger.Log("Running module " + module.moduleNumber + " for 1 second.");

      // Wait for 1 second.
      Timer.delay(1.0);

      // Stop the current module's motor.
      module.stop();
    }
  }

  // used by auto
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    for (SparkSwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
    }
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getYaw(), getPositions(), pose);
  }

  public SparkSwerveModule[] getModules() {
    return mSwerveMods;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < mSwerveMods.length; i++) {
      positions[i] = mSwerveMods[i].getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    // Logger.Log("zeroGyro called");
    gyro.setYaw(180);
  }

  public double getYawValue() {
    return gyro.getYaw().getValueAsDouble();
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - getYawValue())
        : Rotation2d.fromDegrees(getYawValue());
  }

  public enum DrivePosition {
    // add in PID commands later
    MOVELEFT(),
    MOVERIGHT(),
    MOVEFORWARD(),
    MOVEBACK();
  }

  @Override
  public void periodic() {
    odometer.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    for (SparkSwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoderAngle().getDegrees());
    }
    // tell dashboard where the robot thinks it is
    SmartDashboard.putNumber("Robot heading:", getYawValue());
    SmartDashboard.putString("Robot location:", getPose().getTranslation().toString());
  }

  public void stopModules() {
    for (SparkSwerveModule mod : mSwerveMods) {
      mod.stop();
    }
  }

  // constructs current estimate of chassis speeds from module encoders
  public ChassisSpeeds getChassisSpeeds() {
    var frontLeftState = mSwerveMods[0].getState();
    var frontRightState = mSwerveMods[1].getState();
    var backLeftState = mSwerveMods[2].getState();
    var backRightState = mSwerveMods[3].getState();

    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds =
        Constants.Swerve.swerveKinematics.toChassisSpeeds(
            frontLeftState, frontRightState, backLeftState, backRightState);
    return chassisSpeeds;
  }
}
