package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Settings;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.ArmShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.PickUpNote;

public class OperatorControl extends Command {

  private PS4Controller ps4controller;

  private Climber climber;
  private PickUpNote intake;
  private ArmShooter shooter;
  private Arm arm;

  // private SlewRateLimiter translationLimiter = new SlewRateLimiter(10);

  // private ArmPosition L1ArmPosition = ArmPosition.L1;
  // private ArmPosition L2ArmPosition = ArmPosition.L2;
  // private ArmPosition L3ArmPosition = ArmPosition.L3;
  // private ArmPosition L4ArmPosition = ArmPosition.L4;

  long shooterStarted = -1;

  // Make sure the roller imported is the one from subsystems! Not from settings.
  public OperatorControl(Arm arm, Climber climber, PickUpNote pIntake, ArmShooter shooter) {
    addRequirements(climber);
    this.climber = climber;
    this.intake = pIntake;
    this.shooter = shooter;
    this.arm = arm;

    ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);
  }

  @Override
  public void execute() {

    // if (intake.getTimeRunning() > 500 && intake.getAmperage() > 20) {
    // intake.stop();
    // }

    if (ps4controller.getTriangleButton()) {
      arm.setArmPosition(ArmPosition.AMPSHOOTING);
    }

    if (ps4controller.getL1ButtonPressed()) {
      arm.setArmPosition(ArmPosition.INTAKE);
    }

    if (ps4controller.getR1ButtonPressed()) {
      arm.setArmPosition(ArmPosition.REST);
    }

    if (ps4controller.getSquareButtonPressed()) {
      arm.setArmPosition(ArmPosition.SPEAKERSHOOTING);
    }

    if (ps4controller.getCrossButtonPressed()) {
      climber.climb();
      // Logger.Log("climbing now");
    } else if (ps4controller.getCircleButtonPressed()) {
      climber.drop();
      // Logger.Log("dropping now");
    }

    if (ps4controller.getR2Button()) {
      intake.intake();
    } else if (ps4controller.getL2Button()) {
      intake.outtake();
    } else if (!intake.inComplexAction) {
      // not in the middle of complex action
      intake.stop();
    }

    // if (ps4controller.getPSButtonPressed()) {
    // arm.wristEncoder.setPosition(0);
    // }

    // if (ps4controller.getShareButtonPressed()) {
    //   arm.relax();
    // }

    if (ps4controller.getOptionsButtonPressed()) {
      arm.gethard();
      shooter.gethard();
    }

    // the idea here is to run the shooter fo 500ms
    // to get it up to speed, then run the intake for 1000ms
    // then top both of them
    if (ps4controller.getTouchpadPressed()) {
      intake.inComplexAction = true;
      Command c =
          new SequentialCommandGroup(
              new InstantCommand(() -> shooter.shoot()), // run the shooter
              new WaitCommand(2),
              new InstantCommand(() -> intake.intake()), // run the intake
              new WaitCommand(1),
              new InstantCommand(
                  () -> {
                    shooter.stop();
                    intake.stop();
                  })); // stop them both
      c.schedule(); // make it happen!
    }

    if (ps4controller.getPSButtonPressed()) {
      intake.inComplexAction = true;
      Command c =
          new SequentialCommandGroup(
              new InstantCommand(() -> intake.intake()),
              new InstantCommand(() -> shooter.shoot()),
              new WaitCommand(0.5),
              new InstantCommand(
                  () -> {
                    intake.stop();
                    shooter.stop();
                  }));
      c.schedule();
    }

    if (ps4controller.getShareButtonPressed()) {
      Command c =
          new SequentialCommandGroup(
              new InstantCommand(() -> shooter.shoot()),
              new WaitCommand(1),
              new InstantCommand(
                  () -> {
                    shooter.stop();
                  }));
    }

    // Starting to set up mode to control the arm with the left stick
    // double translationVal =
    //     translationLimiter.calculate(MathUtil.applyDeadband(ps4controller.getLeftY(), 0.05));
    // Arm.ArmPosition armPosition = arm.getArmPosition();
    // Arm.ArmPosition newArmPosition = armPosition - (int) Math.round(translationVal * 10);
    // if (Math.abs(translationVal) > 0.1) {
    //   setArmPosition(newArmPosition);
    // }

    // Saves current arm position as the values to be used for the future (only this session)
    // if (ps4controller.getPOV() == 0) {
    //   L1ArmPosition = armPosition;
    // }
    // if (ps4controller.getPOV() == 90) {
    //   L2ArmPosition = armPosition;
    // }
    // if (ps4controller.getPOV() == 180) {
    //   L3ArmPosition = armPosition;
    // }
    // if (ps4controller.getPOV() == 270) {
    //   L4ArmPosition = armPosition;
    // }
  }
}
