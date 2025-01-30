package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class Climber extends SubsystemBase {

    private SparkMax lMotor;
    private SparkMax rMotor;

    private SparkMaxConfig lMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig rMotorConfig = new SparkMaxConfig();

    public RelativeEncoder lEncoder;
    public RelativeEncoder rEncoder;

    private final SparkClosedLoopController lController;
    private final SparkClosedLoopController rController;

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

    public Climber() {

        // Assumption of use of a NEO brushless motor
        lMotor = new SparkMax(Settings.LCLIMBER_MOTOR_ID, MotorType.kBrushless);
        lEncoder = lMotor.getEncoder();
        lController = lMotor.getClosedLoopController();

        rMotor = new SparkMax(Settings.RCLIMBER_MOTOR_ID, MotorType.kBrushless);
        rEncoder = rMotor.getEncoder();
        rController = rMotor.getClosedLoopController();


        // Set the current limits
        lMotorConfig.smartCurrentLimit(80, 80);
        lMotorConfig.secondaryCurrentLimit(85);
        rMotorConfig.smartCurrentLimit(80, 80);
        rMotorConfig.secondaryCurrentLimit(85);

        // // Set the ramp rate since it jumps to full speed too quickly - don't want to
        // // break the robot!
        lMotorConfig.openLoopRampRate(0.2);
        rMotorConfig.openLoopRampRate(0.2);

        // Set the idle mode to brake
        lMotorConfig.idleMode(IdleMode.kBrake);
        rMotorConfig.idleMode(IdleMode.kBrake);

        // Set the CAN timeout to 20ms
        // lMotorConfig.setCANTimeout(20);
        // rMotorConfig.setCANTimeout(20);

        lMotorConfig.voltageCompensation(12.0);
        rMotorConfig.voltageCompensation(12.0);

        // left
        lMotorConfig.closedLoop.p(kP);
        lMotorConfig.closedLoop.i(kI);
        lMotorConfig.closedLoop.d(kD);
        lMotorConfig.closedLoop.iZone(kIz);
        lMotorConfig.closedLoop.velocityFF(kFF);
        // lController.setOutputRange(kMinOutput, kMaxOutput);
        lMotorConfig.closedLoop.maxMotion.maxVelocity(slowVel, smartMotionSlot);
        // lController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        // lController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        // lController.setSmartMotionAllowedClosedLoopError(allowedErr,
        // smartMotionSlot);
        // right
        rMotorConfig.closedLoop.p(kP);
        rMotorConfig.closedLoop.i(kI);
        rMotorConfig.closedLoop.d(kD);
        rMotorConfig.closedLoop.iZone(kIz);
        rMotorConfig.closedLoop.velocityFF(kFF);
        // lController.setOutputRange(kMinOutput, kMaxOutput);
        rMotorConfig.closedLoop.maxMotion.maxVelocity(slowVel, smartMotionSlot);
        // rController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        // rController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        // rController.setSmartMotionAllowedClosedLoopError(allowedErr,
        // smartMotionSlot);

        lEncoder.setPosition(0.0);
        rEncoder.setPosition(0.0);

        // Update the settings
        lMotor.configure(lMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rMotor.configure(rMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void Lintake() {
        lMotor.set(Settings.CLIMBER_INTAKE_SPEED);
        // Logger.Log("lMotor current: " + lMotor.getOutputCurrent());
    }

    public void Rintake() {
        rMotor.set(Settings.CLIMBER_INTAKE_SPEED);
        // Logger.Log("rMotor current: " + lMotor.getOutputCurrent());
    }

    public void Louttake() {
        lMotor.set(Settings.CLIMBER_OUTTAKE_SPEED);
        // Logger.Log("lMotor current: " + lMotor.getOutputCurrent());
    }

    public void Routtake() {
        rMotor.set(Settings.CLIMBER_OUTTAKE_SPEED);
        // Logger.Log("rMotor current: " + lMotor.getOutputCurrent());
    }

    public void leftStop() {
        lMotor.set(0);
        lMotorConfig.idleMode(IdleMode.kBrake);
        lMotor.configure(lMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void rightStop() {
        rMotor.set(0);
        rMotorConfig.idleMode(IdleMode.kBrake);
        rMotor.configure(rMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void climb() {
        lController.setReference(270, SparkMax.ControlType.kPosition);
        rController.setReference(270, SparkMax.ControlType.kPosition);
    }

    public void drop() {
        lController.setReference(0, SparkMax.ControlType.kPosition);
        rController.setReference(0, SparkMax.ControlType.kPosition);
    }

    public void fastDeploy() {
        // increase the max velocity
        lMotorConfig.closedLoop.maxMotion.maxVelocity(fastVel, smartMotionSlot);
        rMotorConfig.closedLoop.maxMotion.maxVelocity(fastVel, smartMotionSlot);
        // drop
        lController.setReference(0, SparkMax.ControlType.kPosition);
        rController.setReference(0, SparkMax.ControlType.kPosition);
        // decrease the max velocity
        rMotorConfig.closedLoop.maxMotion.maxVelocity(slowVel, smartMotionSlot);
        rMotorConfig.closedLoop.maxMotion.maxVelocity(slowVel, smartMotionSlot);

        lMotor.configure(lMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rMotor.configure(rMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}