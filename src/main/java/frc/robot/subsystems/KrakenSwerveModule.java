package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.KrakenModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.Robot;

public class KrakenSwerveModule {
  public String moduleNumber;
  private Rotation2d angleOffset;

  public TalonFX angleMotor;
  public TalonFX driveMotor;
  private CANcoder angleEncoder;
  private TalonFXConfiguration angleConfig;
  private TalonFXConfiguration driveConfig;

  // to limit frequency of printing
  private int printCounter = 0;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  private final PositionVoltage anglePosition = new PositionVoltage(0).withSlot(0);

  public KrakenSwerveModule(String moduleNumber, KrakenModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.encoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new TalonFX(moduleConstants.krakenAngleID);
    angleConfig = new TalonFXConfiguration();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new TalonFX(moduleConstants.krakenDriveID);
    driveConfig = new TalonFXConfiguration();
    configDriveMotor();
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (printCounter % 100 == 0) {
      Logger.Log(
          "setDesiredState()"
              + moduleNumber
              + " speed="
              + desiredState.speedMetersPerSecond
              + " angle="
              + desiredState.angle.getRotations());
    }
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, false);
  }

  private void resetToAbsolute() {
    double canCoderRotations = getCanCoderAngle().getRotations();
    Logger.Log(
        "\n\nangleEncoder"
            + moduleNumber
            + " initMotorPosition: "
            + angleMotor.getPosition().getValueAsDouble());
    Logger.Log("angleEncoder" + moduleNumber + " canCoderPosition: " + canCoderRotations);
    Logger.Log("angleEncoder" + moduleNumber + " angleOffset: " + angleOffset.getRotations());
    double absolutePosition = canCoderRotations - angleOffset.getRotations();
    angleMotor.setPosition(absolutePosition);
    try {
      Thread.sleep(1000);
    } catch (Exception e) {
    }
    Logger.Log("angleEncoder" + moduleNumber + " absolutePosition: " + absolutePosition);
    Logger.Log(
        "angleEncoder"
            + moduleNumber
            + " finalMotorPosition: "
            + angleMotor.getPosition().getValueAsDouble());
    Logger.Log("");
  }

  private void configAngleMotor() {
    //    angleConfig.voltageCompensation(Constants.Swerve.voltageComp);

    angleConfig.MotorOutput.Inverted = Constants.Swerve.angleInverted;
    angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // TEG was Brake

    angleConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.sensorToMechanismRatio;
    angleConfig.ClosedLoopGeneral.ContinuousWrap = Constants.Swerve.continuousWrap;

    angleConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
    angleConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    angleConfig.Slot0.kP = Constants.Swerve.aKrakenKP;
    angleConfig.Slot0.kI = Constants.Swerve.aKrakenKI;
    angleConfig.Slot0.kD = Constants.Swerve.aKrakenKD;
    // angleConfig.Slot0.kS = 0.25;
    // angleConfig.Slot0.kV = 0.12;
    // angleConfig.Slot0.kA = 0.01;

    // need to figure out neutral mode, could be the problem
    // angleMotor.setNeutralMode(NeutralModeValue.valueOf(1));
    angleMotor.getConfigurator().apply(angleConfig);
    resetToAbsolute();
  }

  private void configAngleEncoder() {
    // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.config);
  }

  private void configDriveMotor() {
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // TEG was Brake
    driveConfig.MotorOutput.Inverted = Constants.Swerve.driveInverted;

    driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.sensorToMechanismRatio;
    driveConfig.ClosedLoopGeneral.ContinuousWrap = Constants.Swerve.continuousWrap;

    driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    driveConfig.Slot0.kP = Constants.Swerve.dKrakenKP;
    driveConfig.Slot0.kI = Constants.Swerve.dKrakenKI;
    driveConfig.Slot0.kD = Constants.Swerve.dKrakenKD;

    driveMotor.getConfigurator().apply(driveConfig);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    // if (desiredState.speedMetersPerSecond > 0.01) {
    //   Logger.Log("setSpeed: " + desiredState.speedMetersPerSecond);
    // }
    if (isOpenLoop) {
      // convert speed in m/s to percent output
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.setControl(new DutyCycleOut(percentOutput));
    } else {
      // convert speed  in m/s to motor velocity in rotations per second
      // driveConversionPositionFactor is the number of rotations the motor makes per meter
      double rotationsPerSecond =
          desiredState.speedMetersPerSecond / Constants.Swerve.driveConversionPositionFactor;
      driveMotor.setControl(new VelocityVoltage(rotationsPerSecond));
    }
  }

  public void testDrive(double speed) {
    driveMotor.setControl(new VelocityVoltage(speed));
  }

  private void setAngle(SwerveModuleState desiredState) {
    double oldRotations = angleMotor.getPosition().getValueAsDouble();
    double newRotations = desiredState.angle.getRotations();
    if (Math.abs(newRotations - oldRotations) < 0.02) {
      return;
    }
    angleMotor.setControl(anglePosition.withPosition(newRotations));
  }

  public void testAngle(double speed) {
    angleMotor.setControl(new DutyCycleOut(speed));
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
  }

  public Rotation2d getCanCoderAngle() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public SwerveModuleState getState() {
    // compute wheel speed m/s from the motor velocity in rotations per second
    double speedMetersPerSecond =
        driveMotor.getVelocity().getValueAsDouble()
            * Constants.Swerve.driveConversionPositionFactor;
    Rotation2d angle = getAngle();
    if (printCounter % 100 == 0) {
      Logger.Log(
          "getState(): "
              + moduleNumber
              + " speed="
              + speedMetersPerSecond
              + " angle="
              + angle.getRotations());
    }
    printCounter++;
    return new SwerveModuleState(speedMetersPerSecond, angle);
  }

  public SwerveModulePosition getPosition() {
    double distanceMeters =
        driveMotor.getPosition().getValueAsDouble()
            * Constants.Swerve.driveConversionPositionFactor;
    if (printCounter % 100 == 0) {
      Logger.Log(
          "getPosition(): "
              + moduleNumber
              + " distance="
              + distanceMeters
              + " angle="
              + getAngle().getRotations());
    }
    return new SwerveModulePosition(distanceMeters, getAngle());
  }

  public void stop() {
    driveMotor.setControl(new DutyCycleOut(0));
    angleMotor.setControl(new DutyCycleOut(0));
  }

  public void printCancoderAngle() {
    Logger.Log(
        "Cancoder "
            + moduleNumber
            + ": "
            + (angleEncoder.getAbsolutePosition().getValueAsDouble() - angleOffset.getRotations()));
  }
}
