package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

  private final MotionMagicVoltage position = new MotionMagicVoltage(0);

  public ArmShooter() {

    shooterMotor = new TalonFX(Settings.SHOOTER_MOTOR_ID);
    var talonFXConfigs = new TalonFXConfiguration();


    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40; // was 100, 80, just used 100
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    // below are from https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    
    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    
    shooterMotor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter speed ", shooterMotor.getVelocity().getValueAsDouble());
  }

  // spin the motor 2 rotations forward
  public void intake() {
    double oldRotations = shooterMotor.getPosition().getValueAsDouble(); // current position in rotation
    Logger.Log("intake() got " + oldRotations);
    shooterMotor.setControl(position.withPosition(oldRotations - 5.0));
  }

  // start the motor running to a relative velocity
  public void shoot() {
    shooterMotor.set(-0.5); // 1.0 is full speed
  }

  public void stop() {
    shooterMotor.set(0);
  }

}
