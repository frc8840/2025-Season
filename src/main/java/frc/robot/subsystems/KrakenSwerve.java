package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logger;

public class KrakenSwerve extends SubsystemBase {
  private final Pigeon2 gyro;
  private SwerveDriveOdometry odometer;
  private KrakenSwerveModule[] mSwerveMods;
  private Field2d field;

  public KrakenSwerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    zeroGyro();

    SwerveModulePosition[] startPositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      startPositions[i] = new SwerveModulePosition();
    }
    odometer =
        new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics, new Rotation2d(0), startPositions);

    // Initialize swerve modules with Talon FX Krakens
    mSwerveMods =
        new KrakenSwerveModule[] {
          new KrakenSwerveModule(0, Constants.Swerve.FLKrakenConstants),
          new KrakenSwerveModule(1, Constants.Swerve.FRKrakenConstants),
          new KrakenSwerveModule(2, Constants.Swerve.BLKrakenConstants),
          new KrakenSwerveModule(3, Constants.Swerve.BRKrakenConstants)
        };

    field = new Field2d();
  }

  public void setAngleMotorSpeed(boolean isActive) {
    for (int i=0; i<4; i++) {
      mSwerveMods[i].setAngleMotorSpeed(isActive);
    }
  }

  public void setAngleMotorPosition(double position) {
    for (int i=0; i<4; i++) {
      mSwerveMods[i].setAngleMotorPosition(position);
    }
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

  // used by auto
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    for (KrakenSwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
    }
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getYaw(), getPositions(), pose);
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

  public void zeroGyro() {
    gyro.setYaw(180);
  }

  public double getYawValue() {
    return gyro.getYaw().getValueAsDouble();
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
    }
    // tell dashboard where the robot thinks it is
    // SmartDashboard.putNumber("Robot heading:", getYawValue());
    // SmartDashboard.putString("Robot location:", getPose().getTranslation().toString());
    // for (KrakenSwerveModule mod : mSwerveMods) {
    //   // No voltage being sent to angleMotor, but is being sent to driveMotor
    //   Logger.Log("Module " +  mod.moduleNumber + " Angle Motor Voltage" + mod.angleMotor.getMotorVoltage().getValueAsDouble());
    // }
  }

  public void stopModules() {
    for (KrakenSwerveModule mod : mSwerveMods) {
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
