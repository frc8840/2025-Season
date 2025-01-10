package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class Arm extends SubsystemBase {

    private SparkMaxConfig shoulderConfig = new SparkMaxConfig();
    private SparkMaxConfig wristConfig = new SparkMaxConfig();

    private SparkMax shoulderMotor;
    private SparkMax wristMotor;

    private RelativeEncoder shoulderEncoder;
    public RelativeEncoder wristEncoder;

    SparkClosedLoopController shoulderPID;
    SparkClosedLoopController wristPID;

    private ArmPosition position = ArmPosition.REST;

    public Arm() {

        // set up the motor configs
        // shoulderConfig.restoreFactoryDefaults();
        // shoulderConfig.restoreFactoryDefaults();

        shoulderConfig.idleMode(IdleMode.kBrake);
        wristConfig.idleMode(IdleMode.kBrake);

        shoulderConfig.smartCurrentLimit(80);
        wristConfig.smartCurrentLimit(80);

        shoulderConfig.secondaryCurrentLimit(85);
        wristConfig.secondaryCurrentLimit(85);

        shoulderConfig.closedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE);
        wristConfig.closedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE);

        shoulderConfig.voltageCompensation(12);
        wristConfig.voltageCompensation(12);

        shoulderConfig.encoder.positionConversionFactor((1 / Settings.SHOULDER_GEAR_RATIO) * 360);
        wristConfig.encoder.positionConversionFactor((1 / Settings.WRIST_GEAR_RATIO) * 360);

        // PID configurations

        shoulderConfig.closedLoop.p(Settings.SHOULDER_PID.kP);
        shoulderConfig.closedLoop.i(Settings.SHOULDER_PID.kI);
        shoulderConfig.closedLoop.d(Settings.SHOULDER_PID.kD);
        shoulderConfig.closedLoop.iZone(Settings.SHOULDER_PID.kIZone);
        shoulderConfig.closedLoop.velocityFF(Settings.SHOULDER_PID.kF);

        wristConfig.closedLoop.p(Settings.WRIST_PID.kP);
        wristConfig.closedLoop.i(Settings.WRIST_PID.kI);
        wristConfig.closedLoop.d(Settings.WRIST_PID.kD);
        wristConfig.closedLoop.iZone(Settings.WRIST_PID.kIZone);
        wristConfig.closedLoop.velocityFF(Settings.WRIST_PID.kF);

        // now set up the motors 
        shoulderMotor = new SparkMax(Settings.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        wristMotor = new SparkMax(Settings.WRIST_MOTOR_ID, MotorType.kBrushless);

        shoulderMotor.configure(wristConfig, null, null);
        wristMotor.configure(wristConfig, null, null);
        
        
        // setup the encoders
        shoulderEncoder = shoulderMotor.getEncoder();
        wristEncoder = wristMotor.getEncoder();

        shoulderEncoder.setPosition(0);
        wristEncoder.setPosition(0);

        // the PID controllers
        shoulderPID = shoulderMotor.getClosedLoopController();
        wristPID = wristMotor.getClosedLoopController();

        // shoulderPID.setOutputRange(-Settings.MAX_SHOULDER_SPEED, Settings.MAX_SHOULDER_SPEED);
        // wristPID.setOutputRange(-Settings.MAX_WRIST_SPEED, Settings.MAX_WRIST_SPEED);

        // shoulderMotor.burnFlash();
        // wristMotor.burnFlash();
    }

    public void setArmPosition(ArmPosition position) {
        this.position = position;

        // Logger.Log("shoulder position before:" + shoulderEncoder.getPosition());
        shoulderPID.setReference(
                position.shoulderAngle,
                ControlType.kPosition);

        // elbowPID.setReference(
        // position.elbowAngle,
        // ControlType.kPosition,
        // 0);

        wristPID.setReference(
                position.wristAngle,
                ControlType.kPosition);
    }

    public void relax() {
        shoulderConfig.idleMode(IdleMode.kCoast);
        shoulderMotor.configure(shoulderConfig, null, null);
        shoulderMotor.set(0);
        // elbowMotor.setIdleMode(IdleMode.kCoast);
        // elbowMotor.set(0);
        wristConfig.idleMode(IdleMode.kCoast);
        wristMotor.configure(wristConfig, null, null);
        wristMotor.set(0);
    }

    public void gethard() {
        shoulderConfig.idleMode(IdleMode.kBrake);
        shoulderMotor.configure(shoulderConfig, null, null);

        wristConfig.idleMode(IdleMode.kBrake);
        wristMotor.configure(wristConfig, null, null);
    }

    public ArmPosition getArmPosition() {
        return position;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Shoulder Encoder", shoulderEncoder.getPosition());
        // SmartDashboard.putNumber("Arm/Elbow Encoder", elbowEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist Encoder", wristEncoder.getPosition());
    }

    public enum ArmPosition {
        REST(0, 0),
        INTAKE(0, 117),
        AMPSHOOTING(90, 90),
        SPEAKERSHOOTING(0, 110),
        INTAKEDEMO(0, 117);

        public final double shoulderAngle;
        // public final double elbowAngle;
        public final double wristAngle;

        private ArmPosition(double shoulderAngle, double wristAngle) {
            this.shoulderAngle = shoulderAngle;
            // this.elbowAngle = elbowAngle;
            this.wristAngle = wristAngle;
        }
    }

}