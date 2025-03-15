package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
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

  private TalonFXConfiguration armConfig = new TalonFXConfiguration();

  private TalonFX armMotor;

  private final PositionVoltage shoulderPosition = new PositionVoltage(0).withSlot(0);

  public Arm() {

    // set up the motor configs
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.CurrentLimits.SupplyCurrentLimit = 80;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Settings.CLOSED_LOOP_RAMP_RATE;

    armConfig.Feedback.SensorToMechanismRatio = 36.0 * 3.5; // gearbox is 3*3*4 and chain is 3.5

    // PID configurations
    armConfig.Slot0 = new Slot0Configs();
    armConfig.Slot0.kP = Settings.ARM_PID.kP;
    armConfig.Slot0.kI = Settings.ARM_PID.kI;
    armConfig.Slot0.kD = Settings.ARM_PID.kD;

    // now set up the motor
    armMotor = new TalonFX(Settings.ARM_MOTOR_ID);
    armMotor.getConfigurator().apply(armConfig);
    armMotor.setPosition(0); // assume the arm is in rest position at the start
  }

  public void setArmPositionRotations(double position) {
    // Logger.Log("shoulder position before:" + shoulderEncoder.getPosition());
    // shoulderMotor.setReference(position.shoulderAngle);
    Logger.Log("Shoulder motor thinks it is at " + armMotor.getPosition().getValueAsDouble());
    armMotor.setControl(shoulderPosition.withPosition(position));
    Logger.Log("shoulder position called with:" + position);
  }

  public void relax() {
    Logger.Log("Relaxing arm");
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    armMotor.getConfigurator().apply(armConfig);
    armMotor.set(0);
    // elbowMotor.setIdleMode(IdleMode.kCoast);
    // elbowMotor.set(0);
  }

  public void gethard() {
    Logger.Log("Hardening arm");
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armMotor.getConfigurator().apply(armConfig);
  }

  public double getArmPosition() {
    return armMotor.getPosition().getValueAsDouble(); // returns number in rotations
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Position", armMotor.getPosition().getValueAsDouble());
  }
}
