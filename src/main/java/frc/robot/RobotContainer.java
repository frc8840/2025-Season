package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriverControl;
import frc.robot.commands.OperatorControl;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.KrakenSwerve;
import frc.robot.subsystems.PickUpNote;

public class RobotContainer {
  private static RobotContainer instance;
  private Arm arm;
  public Climber climber;
  private KrakenSwerve swerve;
  public PickUpNote intake;
  public ArmShooter shooter;

  // the old chooser
  // private final SendableChooser<String> oldAutoChooser;
  // // for the choosing stage of pathplanner auto
  // private final SendableChooser<Command> autoChooser;

  // controllers
  DriverControl driverControl;
  OperatorControl operatorControl;

  TrajectoryConfig trajectoryConfig;

  // for autonomous
  // TrajectoryConfig trajectoryConfig;

  public static RobotContainer getInstance() {
    return instance;
  }

  public RobotContainer() {
    instance = this;

    // construct the subsystems
    swerve = new KrakenSwerve();
    // arm = new Arm();
    // climber = new Climber();
    // intake = new PickUpNote();
    // shooter = new ArmShooter();

    Logger.Log("finished constructing subsystems, going to sleep");
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      Logger.Log("sleep interrupted");
    }
    Logger.Log("finished sleeping");

    // now make the controllers
    driverControl = new DriverControl(swerve);
    swerve.setDefaultCommand(driverControl);

    // operatorControl = new OperatorControl(arm, climber, intake, shooter);
    // climber.setDefaultCommand(operatorControl);

    // now we set up things for auto selection and pathplanner
    // // these are commands that the path from pathplanner will use
    // NamedCommands.registerCommand("Start Intake", getStartIntakeCommand());
    // NamedCommands.registerCommand("Stop Intake", getStopIntakeCommand());
    // NamedCommands.registerCommand("Shoot", getShootCommand());

    // The old autonomous chooser
    // oldAutoChooser = new SendableChooser<>();
    // oldAutoChooser.setDefaultOption("Straight", "Straight");
    // oldAutoChooser.setDefaultOption("Left", "Left");
    // oldAutoChooser.setDefaultOption("Right", "Right");
    // oldAutoChooser.setDefaultOption("PathPlanner", "PathPlanner");
    // SmartDashboard.putData("Old Auto Chooser", oldAutoChooser);

    // The new autonomouse chooser
    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    //   newAutoChooser = AutoBuilder.buildAutoChooser();
    //   SmartDashboard.putData("PathPlanner Auto Chooser", newAutoChooser);

    //   trajectoryConfig =
    //       new TrajectoryConfig(
    //               Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    //               Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //           .setKinematics(Constants.Swerve.swerveKinematics);
  }

  // public Command getAutoCommand() {
  //   String oldAutoSelection = oldAutoChooser.getSelected();
  //   if (oldAutoSelection == "PathPlanner") {
  //     return newAutoChooser.getSelected();
  //   }
  //   // otherwise, use the old auto
  //   switch (oldAutoSelection) {
  //     case "Left":
  //       return shootAndDriveForwardCommand(SimpleDirection.diagonalLeft);
  //     case "Right":
  //       return shootAndDriveForwardCommand(SimpleDirection.diagonalRight);
  //     default:
  //       return shootAndDriveForwardCommand(SimpleDirection.straight);
  //   }
  // }

  // public Command shootAndDriveForwardCommand(SimpleDirection direction) {
  //   // get the pose for the direction
  //   Pose2d pose = new Pose2d(0, 0, new Rotation2d(0)); // straight
  //   if (direction == SimpleDirection.diagonalLeft) {
  //     pose = new Pose2d(1.4, 1.4, new Rotation2d(0));
  //   } else if (direction == SimpleDirection.diagonalRight) {
  //     pose = new Pose2d(1.4, 1.4, new Rotation2d(0));
  //   }

  //   intake.inComplexAction = true;
  //   // before we make our trajectory, let it know that we should go in reverse
  //   Trajectory rollForward2Meters =
  //       TrajectoryGenerator.generateTrajectory(
  //           new Pose2d(0, 0, new Rotation2d(0)), List.of(), pose, trajectoryConfig);
  //   return new SequentialCommandGroup(
  //       new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
  //       new InstantCommand(() -> arm.setArmPosition(ArmPosition.SPEAKERSHOOTING)),
  //       new InstantCommand(() -> shooter.shoot()),
  //       new WaitCommand(2),
  //       new InstantCommand(() -> intake.intake()),
  //       new WaitCommand(1),
  //       new InstantCommand(
  //           () -> {
  //             intake.stop();
  //             shooter.stop();
  //           }),
  //       new InstantCommand(() -> arm.setArmPosition(ArmPosition.REST)),
  //       new WaitCommand(2),
  //       getAutonomousCommand(rollForward2Meters),
  //       new InstantCommand(() -> swerve.stopModules()));
  // }

  // public enum SimpleDirection {
  //   straight,
  //   diagonalRight,
  //   diagonalLeft,
  // }

  // public Command getStartIntakeCommand() {
  //   return new InstantCommand(
  //       () -> {
  //         arm.setArmPosition(ArmPosition.INTAKE);
  //         intake.intake();
  //       });
  // }

  // public Command getStopIntakeCommand() {
  //   return new InstantCommand(
  //       () -> {
  //         intake.stop();
  //         arm.setArmPosition(ArmPosition.REST);
  //       });
  // }

  // public Command getShootCommand() {
  //   return new InstantCommand(
  //       () -> {
  //         arm.setArmPosition(ArmPosition.SPEAKERSHOOTING);
  //         shooter.shoot();
  //       });
  // }

  // public SwerveControllerCommand getAutonomousCommand(Trajectory trajectory) {
  //   // create the PID controllers for feedback
  //   PIDController xController = new PIDController(0.2, 0, 0);
  //   PIDController yController = new PIDController(0.2, 0, 0);
  //   ProfiledPIDController thetaController =
  //       new ProfiledPIDController(0.2, 0, 0,
  // Constants.AutoConstants.kThetaControllerConstraints);
  //   ;
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);
  //   return new SwerveControllerCommand(
  //       trajectory,
  //       swerve::getPose,
  //       Constants.Swerve.swerveKinematics,
  //       xController,
  //       yController,
  //       thetaController,
  //       swerve::setModuleStates,
  //       swerve);
  // }

  public Command getPathPlannerAutonomousCommand() {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
