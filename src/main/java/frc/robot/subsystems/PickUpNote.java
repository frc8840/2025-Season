package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.team_8840_lib.info.console.Logger;

public class PickUpNote extends SubsystemBase {

  private SparkMax iMotor;
  public boolean inComplexAction = false;

  private SparkMaxConfig iMotorConfig = new SparkMaxConfig();

  // public long motorStartTime = -1; // not running

  public PickUpNote() {

    iMotor = new SparkMax(Settings.INTAKE_MOTOR_ID, MotorType.kBrushless);
    iMotorConfig.idleMode(IdleMode.kCoast);

    iMotorConfig.smartCurrentLimit(80, 80);
    iMotorConfig.secondaryCurrentLimit(85);

    iMotorConfig.openLoopRampRate(0.2);

    // iMotor.setCANTimeout(20);

    iMotor.configure(iMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Amperage", iMotor.getOutputCurrent());
    boolean noteIn = (Math.abs(iMotor.getOutputCurrent()) > 100);
    SmartDashboard.putBoolean("Intake Successful", noteIn);
  }

  public void intake() {
    iMotor.set(Settings.PICKUP_INTAKE_SPEED);
    // Logger.Log("This is working here");
    // System.out.println("tis is working here");
    Logger.Log("Intake Motor Amperage: " + iMotor.getOutputCurrent());
    // if (motorStartTime < 0) {
    // motorStartTime = System.currentTimeMillis();
    // }
  }

  public void outtake() {
    iMotor.set(Settings.PICKUP_OUTTAKE_SPEED);
    // Logger.Log("This is working here too");
    // System.out.println("tis is working here too");
    Logger.Log("Outtake Motor Amperage: " + iMotor.getOutputCurrent());
  }

  public void stop() {
    iMotor.set(0);
    inComplexAction = false;
    // motorStartTime = -1; // stopped
  }

  public double getAmperage() {
    return iMotor.getOutputCurrent();
  }

  // public long getTimeRunning() {
  // if (motorStartTime < 0)
  // return 0;
  // return System.currentTimeMillis() - motorStartTime;
  // }

}
