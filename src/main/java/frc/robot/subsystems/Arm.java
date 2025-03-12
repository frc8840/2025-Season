package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;

public class Arm extends SubsystemBase {

  private TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();

  private TalonFX shoulderMotor;

  private RelativeEncoder shoulderEncoder;

  // SparkClosedLoopController shoulderPID;

  private ArmPosition position = ArmPosition.REST;

  public Arm() {

    // set up the motor configs

    shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    shoulderConfig.CurrentLimits.SupplyCurrentLimit = 80;
    shoulderConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.supplyCurrentLimitEnable;

    // shoulderConfig.secondaryCurrentLimit(85);

    shoulderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Settings.CLOSED_LOOP_RAMP_RATE;
    // shoulderConfig.closedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE); //was this, not sure if
    // it's right

    // shoulderConfig.voltageCompensation(12);

    // Don't want, this returns natively in rotations and turns it into degrees, which we Don't want
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

    shoulderEncoder.setPosition(0);

    // // the PID controllers
    // shoulderPID = shoulderMotor.getClosedLoopController();

    // shoulderPID.setOutputRange(-Settings.MAX_SHOULDER_SPEED, Settings.MAX_SHOULDER_SPEED);
    // wristPID.setOutputRange(-Settings.MAX_WRIST_SPEED, Settings.MAX_WRIST_SPEED);

  }

  public void setArmPosition(ArmPosition position) {
    this.position = position;

    // Logger.Log("shoulder position before:" + shoulderEncoder.getPosition());
    // shoulderMotor.setReference(position.shoulderAngle);
    shoulderMotor.setPosition(position.shoulderAngle);

    // elbowPID.setReference(
    // position.elbowAngle,
    // ControlType.kPosition,
    // 0);

  }

  public void relax() {
    shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shoulderMotor.getConfigurator().apply(shoulderConfig);
    shoulderMotor.set(0);
    // elbowMotor.setIdleMode(IdleMode.kCoast);
    // elbowMotor.set(0);
  }

  public void gethard() {
    shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shoulderMotor.getConfigurator().apply(shoulderConfig);
  }

  public ArmPosition getArmPosition() {
    return position;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Shoulder Encoder", shoulderEncoder.getPosition());
    // SmartDashboard.putNumber("Arm/Elbow Encoder", elbowEncoder.getPosition());
  }

  public enum ArmPosition {
    REST(0), // was 0,0
    INTAKE(117), // was 0, 117
    AMPSHOOTING(90), // was 90, 90
    SPEAKERSHOOTING(0110), // was 0, 110
    INTAKEDEMO(117); // was 0, 117

    public final double shoulderAngle;

    // public final double elbowAngle;

    private ArmPosition(double shoulderAngle) {
      this.shoulderAngle = shoulderAngle;
      // this.elbowAngle = elbowAngle;
    }
  }
}
