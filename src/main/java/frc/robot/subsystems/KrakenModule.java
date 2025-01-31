package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.team_8840_lib.info.console.Logger;

public class KrakenModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private TalonFXConfiguration angleConfig;
    private TalonFXConfiguration driveConfig;
    
    private CANcoder angleEncoder;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public KrakenModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        angleConfig = new TalonFXConfiguration();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        driveConfig = new TalonFXConfiguration();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, true);
    }

    private void resetToAbsolute() {
        double canAngleDegrees = getCanCoderAngle().getDegrees();
        double absolutePosition = canAngleDegrees - angleOffset.getDegrees();
        angleMotor.setPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.config);
    }

    private void configAngleMotor() {
        angleConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
        angleConfig.MotorOutput.Inverted = Constants.Swerve.angleInvert;
        angleMotor.getConfigurator().apply(angleConfig);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;
        driveConfig.MotorOutput.Inverted = Constants.Swerve.driveInvert;
        driveMotor.getConfigurator().apply(driveConfig);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.setControl(new DutyCycleOut(percentOutput));
        } else {
            driveMotor.setControl(new VelocityVoltage(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.angle.getDegrees() - lastAngle.getDegrees()) <= 1.0)
            return;
        
        angleMotor.setControl(new PositionVoltage(desiredState.angle.getDegrees()));
        lastAngle = desiredState.angle;
    }

    private Rotation2d getAngle() {
        StatusSignal<Double> anglePosition = angleMotor.getPosition();
        return Rotation2d.fromDegrees(anglePosition.getValue());
    }

    public Rotation2d getCanCoderAngle() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModuleState getState() {
        StatusSignal<Double> velocity = driveMotor.getVelocity();
        return new SwerveModuleState(velocity.getValue(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        StatusSignal<Double> position = driveMotor.getPosition();
        return new SwerveModulePosition(position.getValue(), getAngle());
    }

    public void stop() {
        driveMotor.setControl(new DutyCycleOut(0));
        angleMotor.setControl(new DutyCycleOut(0));
    }
}