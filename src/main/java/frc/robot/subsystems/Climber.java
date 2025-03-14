package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;

public class Climber extends SubsystemBase {

  private TalonFX cMotor;

  private TalonFXConfiguration cMotorConfig = new TalonFXConfiguration();

  // private final SparkClosedLoopController lController;
  // private final SparkClosedLoopController rController;

  // constants

  double kP = 0.1;
  double kI = 0.0;
  double kD = 5.0;
  double kIz = 0.0;
  double kFF = 0.0;
  double kMaxOutput = 1;
  double kMinOutput = -1;

  double minVel = 0; // rpm // TODO is this correct?
  double slowVel = 2500; // rpm
  double fastVel = 5000; // rpm
  double maxAcc = 1500;
  double allowedErr = 0; // TODO is this correct?

  ClosedLoopSlot smartMotionSlot = ClosedLoopSlot.kSlot0;

  private final PositionVoltage climberPosition = new PositionVoltage(0).withSlot(0);

  public Climber() {

    // Assumption of use of a NEO brushless motor
    cMotor = new TalonFX(Settings.LCLIMBER_MOTOR_ID);
    // lEncoder = lMotor.getEncoder();
    // lController = lMotor.getClosedLoopController();

    // Set the current limits
    cMotorConfig.CurrentLimits.SupplyCurrentLimit = 80; // was 80, 80,
    cMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;
    // cMotorConfig.secondaryCurrentLimit(85);

    // // Set the ramp rate since it jumps to full speed too quickly - don't want to
    // // break the robot!
    cMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2; // Not sure if this is correct

    // Set the idle mode to brake
    cMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set the CAN timeout to 20ms
    // lMotorConfig.setCANTimeout(20);
    // rMotorConfig.setCANTimeout(20);

    // cMotorConfig.voltageCompensation(12.0);

    cMotorConfig.Slot0.kP = kP;
    cMotorConfig.Slot0.kI = kI;
    cMotorConfig.Slot0.kD = kD;
    // cMotorConfig.closedLoop.iZone(kIz);
    cMotorConfig.Slot0.kV = kFF;
    // lController.setOutputRange(kMinOutput, kMaxOutput);
    // cMotorConfig.closedLoop.maxMotion.maxVelocity(slowVel, smartMotionSlot); //Commented out, not
    // sure if needed
    // lController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    // lController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    // lController.setSmartMotionAllowedClosedLoopError(allowedErr,
    // smartMotionSlot);
    // lController.setOutputRange(kMinOutput, kMaxOutput);

    cMotor.setPosition(0.0);

    // Update the settings
    cMotor.getConfigurator().apply(cMotorConfig);
  }

  public void cIntake() {
    cMotor.set(Settings.CLIMBER_INTAKE_SPEED);
    // Logger.Log("lMotor current: " + lMotor.getOutputCurrent());
  }

  public void cOuttake() {
    cMotor.set(Settings.CLIMBER_OUTTAKE_SPEED);
    // Logger.Log("lMotor current: " + lMotor.getOutputCurrent());
  }

  public void cStop() {
    cMotor.set(0);
    cMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cMotor.getConfigurator().apply(cMotorConfig);
  }

  public void climb() {
    cMotor.setControl(climberPosition.withPosition(0.75)); // was setting to 270 degrees
    // cMotor.setPosition(270);
    // lController.setReference(270, SparkMax.ControlType.kPosition);
    // rController.setReference(270, SparkMax.ControlType.kPosition);
  }

  public void drop() {
    cMotor.setControl(climberPosition.withPosition(0)); // was setting to 0 degrees
    // lController.setReference(0, SparkMax.ControlType.kPosition);
    // rController.setReference(0, SparkMax.ControlType.kPosition);
  }

  public void fastDeploy() {
    // increase the max velocity
    // Set the cruise velocity for Motion Magic (sensor units per 100ms)
    cMotorConfig.MotionMagic.MotionMagicCruiseVelocity = fastVel; // Not sure if right
    // lMotorConfig.closedLoop.maxMotion.maxVelocity(fastVel, smartMotionSlot);
    // drop
    cMotor.setControl(climberPosition.withPosition(0)); // was setting to 0 degrees
    // lController.setReference(0, SparkMax.ControlType.kPosition);
    // decrease the max velocity
    cMotorConfig.MotionMagic.MotionMagicCruiseVelocity = slowVel; // Not sure if right
    // rMotorConfig.closedLoop.maxMotion.maxVelocity(slowVel, smartMotionSlot);

    cMotor.getConfigurator().apply(cMotorConfig);
  }
}
