package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  public TalonFX angleMotor;
  public TalonFX driveMotor;
  private CANcoder angleEncoder;
  private TalonFXConfiguration angleConfig;
  private TalonFXConfiguration driveConfig;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public KrakenSwerveModule(int moduleNumber, KrakenModuleConstants moduleConstants) {
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

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Logger.Log("setDesiredState is running");
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, true);
  }

  private void resetToAbsolute() {
    double canCoderDegrees = getCanCoderAngle().getDegrees();

    Logger.Log("");
    Logger.Log(
        "angleEncoder"
            + angleEncoder.getDeviceID()
            + " initMotorPosition: "
            + angleMotor.getPosition());
    Logger.Log(
        "angleEncoder" + angleEncoder.getDeviceID() + " canCoderDegrees: " + canCoderDegrees);
    Logger.Log(
        "angleEncoder" + angleEncoder.getDeviceID() + " angleOffset: " + angleOffset.getDegrees());
    double absolutePosition = canCoderDegrees - angleOffset.getDegrees();
    angleMotor.setPosition(getOurRotations(absolutePosition));
    try {
      Thread.sleep(1000);
    } catch (Exception e) {
    }

    Logger.Log(
        "angleEncoder" + angleEncoder.getDeviceID() + " absolutePosition: " + absolutePosition);
    Logger.Log(
        "angleEncoder"
            + angleEncoder.getDeviceID()
            + " finalMotorPosition: "
            + angleMotor.getPosition());
    Logger.Log("");
  }

  private void configAngleMotor() {
    angleConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
    angleConfig.MotorOutput.Inverted = Constants.Swerve.angleInverted;
    //need to figure out neutral mode, could be the problem
    // angleMotor.setNeutralMode(NeutralModeValue.valueOf(1));
    angleMotor.getConfigurator().apply(angleConfig);
    resetToAbsolute();
  }

  private void configAngleEncoder() {
    // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.config);
  }

  private void configDriveMotor() {
    driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;
    driveConfig.MotorOutput.Inverted = Constants.Swerve.driveInverted;
    driveMotor.getConfigurator().apply(driveConfig);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.setControl(new DutyCycleOut(percentOutput));
    } else {
      driveMotor.setControl(new VelocityVoltage(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    //this code is somewhat broken
    double targetDegrees = desiredState.angle.getDegrees();
    double currentDegrees = lastAngle.getDegrees();

    // Logger.Log("Module " + moduleNumber + " Target Angle: " + targetDegrees);
    // Logger.Log("Module " + moduleNumber + " Current Angle: " + currentDegrees);

    if (Math.abs(targetDegrees - currentDegrees) <= 1.0) { 
      // Logger.Log("Skipping Small Angle Change");
      return;
    }
    //Prints out and works fine, problem further up the chain, maybe PID values?
    Logger.Log("Module " + moduleNumber + " Voltage Command: " + desiredState.angle.getRotations());

    angleMotor.setControl(new PositionVoltage(desiredState.angle.getRotations()));
    lastAngle = desiredState.angle;
  }

//Works unlike setTestAngle
  private void setTestAngle(SwerveModuleState desiredState) {
    angleMotor.setControl(new DutyCycleOut(0.2));
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(angleMotor.getPosition().getValueAsDouble());
  }

  public Rotation2d getCanCoderAngle() {
    return Rotation2d.fromRotations(angleEncoder.getPosition().getValueAsDouble());
  }

  private double getOurRotations(double degrees) {
    return degrees / 360.0;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), getAngle());
  }

  public void stop() {
    driveMotor.setControl(new DutyCycleOut(0));
    angleMotor.setControl(new DutyCycleOut(0));
  }
}
