package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.Settings;

public class Arm extends SubsystemBase {

  private TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();

  private TalonFX shoulderMotor;

  // SparkClosedLoopController shoulderPID;

  private double position = 0;

  private final PositionVoltage shoulderPosition = new PositionVoltage(0).withSlot(0);

  public Arm() {

    // set up the motor configs

    shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    shoulderConfig.CurrentLimits.SupplyCurrentLimit = 80;
    shoulderConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.supplyCurrentLimitEnable;

    // shoulderConfig.secondaryCurrentLimit(85);

    shoulderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Settings.CLOSED_LOOP_RAMP_RATE;

    shoulderConfig.Feedback.SensorToMechanismRatio = 36.0 * 3.5; // gearbox is 3*3*4 and chain is 3.5
    // shoulderConfig.closedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE); //was this, not sure if
    // it's right

    // shoulderConfig.voltageCompensation(12);

    // Don't want, this takes something in rotations and turns it into degrees, which we don't want
    // to do.
    // shoulderConfig.encoder.positionConversionFactor((1 / Settings.SHOULDER_GEAR_RATIO) * 360);

    // PID configurations

    shoulderConfig.Slot0.kP = Settings.SHOULDER_PID.kP;
    shoulderConfig.Slot0.kI = Settings.SHOULDER_PID.kI;
    shoulderConfig.Slot0.kD = Settings.SHOULDER_PID.kD;
    // shoulderConfig.Slot0.kV = Settings.SHOULDER_PID.kF; //Feedforward I think, not sure if needed

    shoulderMotor = new TalonFX(Settings.SHOULDER_MOTOR_ID);

    // now set up the motors
    shoulderMotor.getConfigurator().apply(shoulderConfig);

    // setup the encoders
    // shoulderEncoder = shoulderMotor.getEncoder();
    // shoulderMotor.

    shoulderMotor.setPosition(0);

    // // the PID controllers
    // shoulderPID = shoulderMotor.getClosedLoopController();

    // shoulderPID.setOutputRange(-Settings.MAX_SHOULDER_SPEED, Settings.MAX_SHOULDER_SPEED);
    // wristPID.setOutputRange(-Settings.MAX_WRIST_SPEED, Settings.MAX_WRIST_SPEED);

  }

  public void setArmPosition(double position) {
    // Logger.Log("shoulder position before:" + shoulderEncoder.getPosition());
    // shoulderMotor.setReference(position.shoulderAngle);
    Logger.Log("Shoulder motor thinks it is at " + shoulderMotor.getPosition().getValueAsDouble());
    shoulderMotor.setControl(shoulderPosition.withPosition(position));
    Logger.Log("shoulder position called with:" + position);

    // elbowPID.setReference(
    // position.elbowAngle,
    // ControlType.kPosition,
    // 0);

  }

  public void relax() {
    Logger.Log("Relaxing arm");
    shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shoulderMotor.getConfigurator().apply(shoulderConfig);
    shoulderMotor.set(0);
    // elbowMotor.setIdleMode(IdleMode.kCoast);
    // elbowMotor.set(0);
  }

  public void gethard() {
    Logger.Log("Hardening arm");
    shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shoulderMotor.getConfigurator().apply(shoulderConfig);
  }

  public double getArmPosition() {
    return shoulderMotor.getPosition().getValueAsDouble(); // returns number in rotations
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Arm/Shoulder Encoder", shoulderMotor.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Arm/Elbow Encoder", elbowEncoder.getPosition());
  }

  // REST(0), // was 0,0, changing to rotations
  //   INTAKE(0.325), // was 0, 117, changing to rotations
  //   AMPSHOOTING(0.25), // was 90, 90, changing to rotations
  //   SPEAKERSHOOTING(0.306), // was 0, 110, changing to rotations
  //   INTAKEDEMO(0.325); // was 0, 117, changing to rotations

}
