// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.config.CTREConfigs;
import frc.robot.subsystems.KrakenSwerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static Robot instance;
  public static CTREConfigs ctreConfigs;

  RobotContainer container;
  KrakenSwerve swerve;
  private LaserCan lc;

  public static Robot getInstance() {
    return instance;
  }

  public Robot() {
    instance = this;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.Log("RobotInitRan");
    ctreConfigs = new CTREConfigs();
    container = new RobotContainer();
    swerve = new KrakenSwerve();
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    lc = new LaserCan(0);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in
    // GrappleHook
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Logger.loopCounter++;
    CommandScheduler.getInstance().run();
    // CanBridge.runTCP();
    LaserCan.Measurement measurement = lc.getMeasurement();
    // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
    //   System.out.println("The target is " + measurement.distance_mm + "mm away!");
    // } else {
    //   System.out.println(
    //       "Oh no! The target is out of range, or we can't get a reliable measurement!");
    //   // You can still use distance_mm in here, if you're ok tolerating a clamped value or an
    //   // unreliable measurement.
    // }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // Logger.Log("Autonomous Init Called");
    Command autonomousCommand = swerve.followPathCommand("Test Path");
    // schedule the autonomous command - adds it to the scheduler
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // container.shooter.stop();
    // container.intake.stop();
  }

  // Line 104 to 120 can be commented out, js don't command out teleopPeriodic
  //   private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
  //    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10%
  // deadband
  //    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  //    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  // private final XboxController m_joystick = new XboxController(0);
  // public final SwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //  m_drivetrain.setControl(
    //     m_driveRequest.withVelocityX(-joystick.getLeftY())
    //        .withVelocityY(-joystick.getLeftX())
    //        .withRotationalRate(-joystick.getRightX())
    //  );
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
