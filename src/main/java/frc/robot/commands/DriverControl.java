package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.subsystems.KrakenSwerve;

public class DriverControl extends Command {

  private XboxController xboxcontroller;
  private KrakenSwerve swerve;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(10);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(10);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(10);

  private boolean isReefRotating = false;
  private boolean isHuggingReef = false;

  // Make sure the roller imported is the one from subsystems! Not from settings.
  public DriverControl(KrakenSwerve swerve) {
    addRequirements(swerve);

    this.swerve = swerve;
    xboxcontroller = new XboxController(Settings.DRIVER_CONTROLLER_PORT);
  }

  @Override
  public void execute() {

    if (xboxcontroller.getXButtonPressed()) {
      swerve.zeroGyro();
    }

    if (xboxcontroller.getYButtonPressed()) {
      swerve.printCancoderAngles();
    }

    if (xboxcontroller.getBButtonPressed()) {
      swerve.stopModules();
    }

    if (xboxcontroller.getAButtonPressed()) {
      isReefRotating = !isReefRotating;
      if (!isReefRotating) {
        swerve.rotateAroundReef(false, 0.0);
      }
    }
    if (xboxcontroller.getBackButtonPressed()) {
      isHuggingReef = !isHuggingReef;
    }
    // get values from the Xbox Controller joysticks
    // apply the deadband so we don't do anything right around the center of the
    // joysticks
    double translationVal =
        translationLimiter.calculate(MathUtil.applyDeadband(-xboxcontroller.getLeftY(), 0.05));
    double strafeVal =
        strafeLimiter.calculate(MathUtil.applyDeadband(xboxcontroller.getLeftX(), 0.05));
    double rotationVal =
        rotationLimiter.calculate(MathUtil.applyDeadband(xboxcontroller.getRightX(), 0.05));

    /* Drive */
    if (!isReefRotating) {
      Logger.LogPeriodic(
          "swerve.drive() called with translation=" + translationVal + " and strafe=" + strafeVal);
      swerve.drive(
          new Translation2d(translationVal, strafeVal)
              .times(Constants.Swerve.maxSpeedMetersPerSecond), // convert to m/s
          rotationVal * Constants.Swerve.maxAngularVelocityRadiansPerSecond,
          false);
      // ask for ChassisSpeeds so we can print it to logs for debugging
      ChassisSpeeds chassisSpeeds = swerve.getChassisSpeeds();
      Logger.LogPeriodic("getChassisSpeeds: " + chassisSpeeds);
    } else {
      if (isHuggingReef) {
        swerve.rotateAroundReef(true, translationVal);
      } else {
        swerve.rotateAroundReef(false, strafeVal);
        ;
      }
    }
  }
}
