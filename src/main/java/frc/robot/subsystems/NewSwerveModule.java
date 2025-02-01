package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.team_8840_lib.info.console.Logger;

public class NewSwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SparkMax angleMotor;
    private SparkMaxConfig angleConfig;
    private SparkMax driveMotor;
    private SparkMaxConfig driveConfig;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController angleController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public NewSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getClosedLoopController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Custom optimize command, since default WPILib optimize assumes continuous
        // controller which
        // REV and CTRE are not

        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, true); // let's try open loop control for now
    }

    private void resetToAbsolute() {

        double canAngleDegrees = getCanCoderAngle().getDegrees();

        Logger.Log("");
        Logger.Log("angleEncoder" + angleEncoder.getDeviceID() + " initMotorPosition: " + integratedAngleEncoder.getPosition());
        Logger.Log("angleEncoder" + angleEncoder.getDeviceID() + " canCoderDegrees: " + canAngleDegrees);
        Logger.Log("angleEncoder" + angleEncoder.getDeviceID() + " angleOffset: " + angleOffset.getDegrees());
        double absolutePosition = canAngleDegrees - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
        try {
            Thread.sleep(1000);
        } catch (Exception e) {
        }
        
        Logger.Log("angleEncoder" + angleEncoder.getDeviceID() + " absolutePosition: " + absolutePosition);
        Logger.Log("angleEncoder" + angleEncoder.getDeviceID() + " finalMotorPosition: " + integratedAngleEncoder.getPosition());
        Logger.Log("");
    }

    private void configAngleEncoder() {
        // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.config);
    }

    private void configAngleMotor() {
        angleConfig = new SparkMaxConfig();
        angleConfig.smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        angleConfig.inverted(Constants.Swerve.angleInvert);
        angleConfig.idleMode(Constants.Swerve.angleNeutralMode);
        angleConfig.encoder.positionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleConfig.closedLoop.p(Constants.Swerve.angleKP);
        angleConfig.closedLoop.i(Constants.Swerve.angleKI);
        angleConfig.closedLoop.d(Constants.Swerve.angleKD);
        angleConfig.closedLoop.velocityFF(Constants.Swerve.angleKFF);
        angleConfig.voltageCompensation(Constants.Swerve.voltageComp);
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // is this correct?
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveConfig = new SparkMaxConfig();
        driveConfig.smartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveConfig.inverted(Constants.Swerve.driveInvert);
        driveConfig.idleMode(Constants.Swerve.driveNeutralMode);
        driveConfig.encoder.velocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        driveConfig.encoder.positionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        driveConfig.closedLoop.p(Constants.Swerve.angleKP);
        driveConfig.closedLoop.i(Constants.Swerve.angleKI);
        driveConfig.closedLoop.d(Constants.Swerve.angleKD);
        driveConfig.closedLoop.velocityFF(Constants.Swerve.angleKFF);
        driveConfig.voltageCompensation(Constants.Swerve.voltageComp);
        driveEncoder.setPosition(0.0);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        if (Math.abs(desiredState.angle.getDegrees() - lastAngle.getDegrees()) <= 1.0)
            return;

        // Logger.Log("desiredAngle: " + desiredState.angle.getDegrees() + "
        // currentAngle: " + getAngle().getDegrees());

        angleController.setReference(desiredState.angle.getDegrees(), ControlType.kPosition);
        lastAngle = desiredState.angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoderAngle() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

}