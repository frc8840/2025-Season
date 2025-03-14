package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;

public class ArmShooter extends SubsystemBase {

  public TalonFX shooterMotor;

  private TalonFXConfiguration lMotorConfig = new TalonFXConfiguration();
  // private TalonFXConfiguration rMotorConfig = new TalonFXConfiguration();

  public boolean inShooterComplexAction = false;

  public ArmShooter() {

    shooterMotor = new TalonFX(Settings.SHOOTER_MOTOR_ID);
    // rightMotor = new TalonFX(Settings.SHOOTER_MOTOR_ID2);

    lMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // rMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    lMotorConfig.CurrentLimits.SupplyCurrentLimit = 100; // was 100, 80, just used 100
    lMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    // lMoterConfig.CurrentLimits.secondaryCurrentLimit(105); //Doesn't exist for TalonFX

    // rMotorConfig.CurrentLimits.SupplyCurrentLimit = 100; // was 100, 80, just used 100
    // rMoterConfig.secondaryCurrentLimit(105); //Doesn't exist for TalonFX

    // sMotor.setOpenLoopRampRate(0.2);

    lMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;

    // sMotor2.setOpenLoopRampRate(0.2);

    // lMoterConfig.setCANTimeout(20);
    // rMoterConfig.setCANTimeout(20);

    shooterMotor.getConfigurator().apply(lMotorConfig);
    // rightMotor.getConfigurator().apply(lMotorConfig);
    // leftMotor.configure(
    //     lMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // rightMotor.configure(
    //     rMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter speed ", shooterMotor.getVelocity().getValueAsDouble());
  }

  public void shoot() {
    shooterMotor.set(Settings.SHOOTER_OUT_SPEED);
    // rightMotor.set(-Settings.SHOOTER_OUT_SPEED);
  }

  public void stop() {
    shooterMotor.set(0);
    // rightMotor.set(0);
    inShooterComplexAction = false;
  }

  public void gethard() {
    lMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // rMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterMotor.getConfigurator().apply(lMotorConfig);
    // rightMotor.getConfigurator().apply(lMotorConfig);
  }

  public boolean isAbletoShoot() {
    return true;
  }
}
