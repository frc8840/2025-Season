package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class ArmShooter extends SubsystemBase {

    public SparkMax leftMotor;
    public SparkMax rightMotor;
    
    private SparkMaxConfig lMoterConfig = new SparkMaxConfig();
    private SparkMaxConfig rMoterConfig = new SparkMaxConfig();

    public RelativeEncoder leftEncoder;
    public RelativeEncoder rightEncoder;

    public boolean inShooterComplexAction = false;

    public ArmShooter() {

        leftMotor = new SparkMax(Settings.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Settings.SHOOTER_MOTOR_ID2, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        // leftMotor.restoreFactoryDefaults();
        // rightMotor.restoreFactoryDefaults();

        lMoterConfig.idleMode(IdleMode.kCoast);
        rMoterConfig.idleMode(IdleMode.kCoast);

        lMoterConfig.smartCurrentLimit(100, 80);
        lMoterConfig.secondaryCurrentLimit(105);

        rMoterConfig.smartCurrentLimit(100, 80);
        rMoterConfig.secondaryCurrentLimit(105);

        // sMotor.setOpenLoopRampRate(0.2);
        // sMotor2.setOpenLoopRampRate(0.2);

        // lMoterConfig.setCANTimeout(20);
        // rMoterConfig.setCANTimeout(20);

        // lMoterConfig.burnFlash();
        // rMoterConfig.burnFlash();

        leftMotor.configure(lMoterConfig, null, null);
        rightMotor.configure(rMoterConfig, null, null);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter left ", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter right ", rightEncoder.getVelocity());
        boolean isReady = (Math.abs(leftEncoder.getVelocity()) > 4300 && Math.abs(rightEncoder.getVelocity()) > 4300);
        SmartDashboard.putBoolean("Shooter Ready", isReady);

    }

    public void shoot() {
        leftMotor.set(Settings.SHOOTER_OUT_SPEED);
        rightMotor.set(-Settings.SHOOTER_OUT_SPEED);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
        inShooterComplexAction = false;
    }

    public void gethard() {
        lMoterConfig.idleMode(IdleMode.kBrake);
        rMoterConfig.idleMode(IdleMode.kBrake);
    }

    public boolean isAbletoShoot() {
        return true;
    }
}

