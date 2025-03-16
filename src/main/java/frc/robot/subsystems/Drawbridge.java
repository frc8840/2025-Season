package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class Drawbridge extends SubsystemBase {

  private SparkMax motor;
  private SparkMaxConfig config;
  private RelativeEncoder encoder;
  private SparkClosedLoopController motorController;

  public Drawbridge() {

    motor = new SparkMax(Settings.INTAKE_MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    config = new SparkMaxConfig();
    config.smartCurrentLimit(20);
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(1.0);
    config.closedLoop.p(1.0);
    config.closedLoop.i(0);
    config.closedLoop.d(0);
    config.closedLoop.velocityFF(0);
    config.voltageCompensation(12.0);
    motor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // is this correct?
    motorController = motor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Motor Position", encoder.getPosition());
  }

  public void open() {
    motorController.setReference(10, ControlType.kPosition);
  }

  public void close() {
    motorController.setReference(0, ControlType.kPosition);
  }
}
