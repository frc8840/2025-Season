package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.KrakenModuleConstants;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KrakenControl extends SubsystemBase {
    TalonSRX motor = new TalonSRX(1); // Initialize your motor controller with the appropriate device ID
    private final Pigeon2 gyro;
    private SwerveDriveOdometry odometer;
    private KrakenModuleConstants[] mSwerveMods;
    private Field2d field;

    public KrakenControl() {
        motor.setNeutralMode(NeutralMode.Coast); // Set the motor to coast mode
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        zeroGyro();

        SwerveModulePosition[] startPositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            startPositions[i] = new SwerveModulePosition();
        }
        odometer = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, new Rotation2d(0), startPositions);

        // Initialize swerve modules with Talon FX Krakens
        mSwerveMods = new KrakenModuleConstants[] {
                new KrakenModule(0, Constants.Swerve.FLKrakenConstants),
                new KrakenModule(1, Constants.Swerve.FRKrakenConstants),
                new KrakenModule(2, Constants.Swerve.BLKrakenConstants),
                new KrakenModule(3, Constants.Swerve.BRKrakenConstants)
        };

    }

    public void runMotor(double speed) {
        motor.set(ControlMode.PercentOutput, speed); // Set the motor to run at the specified speed
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for (NewSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber]);
        }
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getYaw(), getPositions(), pose);
    }

    public NewSwerveModule[] getModules() {
        return mSwerveMods;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < mSwerveMods.length; i++) {
            positions[i] = mSwerveMods[i].getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(180);
    }

    public double getYawValue() {
        return gyro.getYaw().getValueAsDouble();
    }

    public Rotation2d getYaw() {
        return Constants.Swerve.invertGyro ? Rotation2d.fromDegrees(360 - getYawValue()) : Rotation2d.fromDegrees(getYawValue());
    }

    @Override
    public void periodic() {
        odometer.update(getYaw(), getPositions());
        field.setRobotPose(getPose());
        SmartDashboard.putNumber("Robot heading:", getYawValue());
        SmartDashboard.putString("Robot location:", getPose().getTranslation().toString());
    }
}