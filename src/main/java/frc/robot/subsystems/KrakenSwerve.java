package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

public class KrakenSwerve extends SubsystemBase {
  private final AHRS gyro;
  private SwerveDriveOdometry odometer;
  private KrakenSwerveModule[] mSwerveMods;
  private Field2d field;
  private int printCounter = 0;

  public KrakenSwerve() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    zeroGyro();

    SwerveModulePosition[] startPositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      startPositions[i] = new SwerveModulePosition();
    }
    // is it true that every module is at angle 0.0 when we start?
    odometer =
        new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics, new Rotation2d(0), startPositions);

    // Initialize swerve modules with Talon FX Krakens
    mSwerveMods =
        new KrakenSwerveModule[] {
          new KrakenSwerveModule("FL", Constants.Swerve.FLKrakenConstants),
          new KrakenSwerveModule("FR", Constants.Swerve.FRKrakenConstants),
          new KrakenSwerveModule("BL", Constants.Swerve.BLKrakenConstants),
          new KrakenSwerveModule("BR", Constants.Swerve.BRKrakenConstants)
        };

    field = new Field2d();

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveFromSpeeds(
                speeds), // Method that will drive the robot given ROBOT RELATIVE(speeds), //
        // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds. Also optionally outputs individual module
        // feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(0.2, 0.0, 0.0), // Translation PID constants
            new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
            ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
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
  }

  // translation and rotation are the desired behavior of the robot at this moment
  // translation vector is the desired velocity in m/s and
  // rotation is the desired angular velocity in rotations per second
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    // first, we compute our desired chassis speeds
    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    driveFromSpeeds(chassisSpeeds);
  }

  // used by DriverControl and AutoBuilder
  public void driveFromSpeeds(ChassisSpeeds speeds) {
    if (printCounter % 100 == 0) {
      Logger.Log(
          "driveFromSpeeds() called with "
              + speeds.vxMetersPerSecond
              + ","
              + speeds.vyMetersPerSecond
              + " and "
              + speeds.omegaRadiansPerSecond);
    }
    printCounter++;
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
    // do we need the below?
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    setModuleStates(swerveModuleStates);
  }

  // for testing
  public void runEachMotorForOneSecond() {
    for (KrakenSwerveModule module : mSwerveMods) {
      // Run the current module's motor at full speed (or desired speed).
      module.setDesiredState(new SwerveModuleState(Constants.Swerve.maxSpeed, new Rotation2d()));
      Logger.Log("Running module " + module.moduleNumber + " for 1 second.");

      // Wait for 1 second.
      Timer.delay(1.0);

      // Stop the current module's motor.
      module.stop();
    }
  }

  public void testDriveForOneSec() {
    for (KrakenSwerveModule module : mSwerveMods) {
      // Run the current module's motor at full speed (or desired speed).
      module.testDrive(0.2);
    }
  }

  public void testAngleMotors() {
    for (KrakenSwerveModule module : mSwerveMods) {
      // Run the current module's motor at full speed (or desired speed).
      module.testAngle(0.2);
      Logger.Log("Running module " + module.moduleNumber + " for 1 second.");

      // Wait for 1 second.
      Timer.delay(1.0);

      // Stop the current module's motor.
      module.testAngle(0);
    }
  }

  public void rotateAroundReef(boolean hugReef, double speed) {
    if (hugReef) {
      mSwerveMods[0].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.164))); // FL
      mSwerveMods[1].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.275))); // FR
      mSwerveMods[2].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.336))); // BL
      mSwerveMods[3].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.442))); // BR
    } else {
      // Changing to be orthogonal to the current rotation pattern. should make it closer/farther
      // from the reef
      mSwerveMods[0].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.664))); // FL
      mSwerveMods[1].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.775))); // FR
      mSwerveMods[2].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.836))); // BL
      mSwerveMods[3].setDesiredState(
          new SwerveModuleState(speed, Rotation2d.fromRotations(0.942))); // BR
    }
  }

  public void printCancoderAngles() {
    for (KrakenSwerveModule module : mSwerveMods) {
      // Run the current module's motor at full speed (or desired speed).
      module.printCancoderAngle();
    }
  }

  // used by auto
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    for (int i = 0; i < mSwerveMods.length; i++) {
      mSwerveMods[i].setDesiredState(desiredStates[i]);
    }
  }

  public Pose2d getPose() {
    if (printCounter % 1000 == 0) {
      Logger.Log("Pose is " + odometer.getPoseMeters());
    }
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getYaw(), getPositions(), pose);
  }

  public void zeroOdometry() {
    Rotation2d zeroedPosition = new Rotation2d(0);
    odometer.update(zeroedPosition, getPositions());
  }

  public KrakenSwerveModule[] getModules() {
    return mSwerveMods;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < mSwerveMods.length; i++) {
      positions[i] = mSwerveMods[i].getPosition();
    }
    return positions;
  }

  public SwerveModulePosition[] getZeroedPositions() {
    SwerveModulePosition[] zeroPositions = new SwerveModulePosition[4];
    for (int i = 0; i < mSwerveMods.length; i++) {
      zeroPositions[i] = new SwerveModulePosition(0.0, new Rotation2d(0.0));
    }
    return zeroPositions;
  }

  public void zeroGyro() {
    gyro.setAngleAdjustment(0);
  }

  public double getYawValue() {
    return gyro.getYaw();
  }

  public Rotation2d getYaw() {
    return Constants.Swerve.invertGyro
        ? Rotation2d.fromDegrees(360 - getYawValue())
        : Rotation2d.fromDegrees(getYawValue());
  }

  @Override
  public void periodic() {
    odometer.update(getYaw(), getPositions());
    field.setRobotPose(getPose());
    for (KrakenSwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoderAngle().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " StateAngle", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Speed", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Distance", mod.getPosition().distanceMeters);
    }
    // tell dashboard where the robot thinks it is
    SmartDashboard.putNumber("Robot heading:", getYawValue());
    SmartDashboard.putString("Robot location:", getPose().getTranslation().toString());
    SmartDashboard.putString("Module Positions: ", getPositions().toString());
    // for (KrakenSwerveModule mod : mSwerveMods) {
    //   // No voltage being sent to angleMotor, but is being sent to driveMotor
    //   Logger.Log("Module " +  mod.moduleNumber + " Angle Motor Voltage" +
    // mod.angleMotor.getMotorVoltage().getValueAsDouble());
    // }
  }

  public void stopModules() {
    for (KrakenSwerveModule mod : mSwerveMods) {
      mod.stop();
    }
  }

  // constructs current estimate of chassis speeds from module encoders
  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState frontLeftState = mSwerveMods[0].getState();
    SwerveModuleState frontRightState = mSwerveMods[1].getState();
    SwerveModuleState backLeftState = mSwerveMods[2].getState();
    SwerveModuleState backRightState = mSwerveMods[3].getState();

    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds =
        Constants.Swerve.swerveKinematics.toChassisSpeeds(
            frontLeftState, frontRightState, backLeftState, backRightState);

    return chassisSpeeds;
  }
}
