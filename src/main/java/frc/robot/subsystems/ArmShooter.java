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

public class ArmShooter extends SubsystemBase {

  public TalonFX shooterMotor;

  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final PositionVoltage position = new PositionVoltage(0).withSlot(0);

  public ArmShooter() {

    shooterMotor = new TalonFX(Settings.SHOOTER_MOTOR_ID);

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 40; // was 100, 80, just used 100
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2; // for getting up to speed

    // PID for position control
    motorConfig.Slot0 = new Slot0Configs();
    motorConfig.Slot0.kP = 1.0;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;

    shooterMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter speed ", shooterMotor.getVelocity().getValueAsDouble());
  }

  // spin the motor 2 rotations forward
  public void intake() {
    double oldRotations = shooterMotor.getPosition().getValueAsDouble(); // current position in rotation
    Logger.Log("intake() got " + oldRotations);
    shooterMotor.setControl(position.withPosition(oldRotations + 5.0));
  }

  // start the motor running to a relative velocity
  public void shoot() {
    shooterMotor.set(-0.5); // 1.0 is full speed
  }

  public void stop() {
    shooterMotor.set(0);
  }

}
