package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.Settings;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX iMotor;
  public boolean inComplexAction = false;

  private TalonFXConfiguration iMotorConfig = new TalonFXConfiguration();

  // public long motorStartTime = -1; // not running

  public IntakeSubsystem() {

    iMotor = new TalonFX(Settings.INTAKE_MOTOR_ID);

    iMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    iMotorConfig.CurrentLimits.SupplyCurrentLimit = 80; // was 80, 80,
    iMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.supplyCurrentLimitEnable;

    // iMotorConfig.secondaryCurrentLimit(85);

    iMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2; // Not sure if this is correct

    // iMotor.setCANTimeout(20);

    iMotor.getConfigurator().apply(iMotorConfig);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Intake Amperage",
        iMotor
            .getSupplyCurrent()
            .getValueAsDouble()); // was getInput Current, not sure if this is the parallel
    boolean noteIn =
        (Math.abs(iMotor.getStatorCurrent().getValueAsDouble())
            > 100); // was getOutput Current, not sure if this is the parallel
    SmartDashboard.putBoolean("Intake Successful", noteIn);
  }

  public void intake() {
    iMotor.set(Settings.PICKUP_INTAKE_SPEED);
    // Logger.Log("This is working here");
    // System.out.println("tis is working here");
    Logger.Log(
        "Intake Motor Amperage: "
            + iMotor
                .getStatorCurrent()
                .getValueAsDouble()); // was getOutput Current, not sure if this is the parallel
    // if (motorStartTime < 0) {
    // motorStartTime = System.currentTimeMillis();
    // }
  }

  public void outtake() {
    iMotor.set(Settings.PICKUP_OUTTAKE_SPEED);
    // Logger.Log("This is working here too");
    // System.out.println("tis is working here too");
    Logger.Log(
        "Outtake Motor Amperage: "
            + iMotor
                .getStatorCurrent()
                .getValueAsDouble()); // was getOutput Current, not sure if this is the parallel
  }

  public void stop() {
    iMotor.set(0);
    inComplexAction = false;
    // motorStartTime = -1; // stopped
  }

  public double getAmperage() {
    return iMotor
        .getStatorCurrent()
        .getValueAsDouble(); // was getOutput Current, not sure if this is the parallel
  }

  // public long getTimeRunning() {
  // if (motorStartTime < 0)
  // return 0;
  // return System.currentTimeMillis() - motorStartTime;
  // }

}
