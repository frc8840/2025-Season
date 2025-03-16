package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmShooter;
import frc.robot.subsystems.Drawbridge;

public class OperatorControl extends Command {

  private PS4Controller ps4controller;

  private Arm arm;
  private ArmShooter shooter;
  private Drawbridge drawbridge;

  public OperatorControl(Arm arm, ArmShooter shooter, Drawbridge drawbridge) {
    this.arm = arm;
    this.shooter = shooter;
    this.drawbridge = drawbridge;
    addRequirements(arm);
    addRequirements(shooter);
    addRequirements(drawbridge);

    ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);
  }

  @Override
  public void execute() {

    // drawbridge related
    if (ps4controller.getPOV() == 0) {
      Logger.Log("POV top button pressed");
      drawbridge.open();
    } else if (ps4controller.getPOV() == 180) {
      Logger.Log("POV bottom button pressed");
      drawbridge.close();
    }

    // shooter/intake related
    if (ps4controller.getL2ButtonPressed()) {
      Logger.Log("L2 button pressed");
      shooter.intake();
    }
    if (ps4controller.getL1ButtonPressed()) {
      Logger.Log("L1 button pressed");
      shooter.outtake();
    }
    if (ps4controller.getR2Button()) {
      shooter.runForward();
    } else if (ps4controller.getR1Button()) {
      shooter.runBackward();
    } else {
      shooter.stop();
    }

    // arm position related
    if (ps4controller.getTriangleButtonPressed()) {
      Logger.Log("Triangle button pressed");
      arm.setArmPositionRotations(-20.0); // high
      Logger.Log("Arm position: " + arm.getArmPosition());
    }

    if (ps4controller.getSquareButtonPressed()) {
      Logger.Log("Square button pressed");
      arm.setArmPositionRotations(-18.0); // medium
      Logger.Log("Arm position: " + arm.getArmPosition());
    }

    if (ps4controller.getCircleButtonPressed()) {
      Logger.Log("Square button pressed");
      arm.setArmPositionRotations(-12.0); // medium
      Logger.Log("Arm position: " + arm.getArmPosition());
    }

    if (ps4controller.getCrossButtonPressed()) {
      Logger.Log("Cross button pressed");
      arm.setArmPositionRotations(0); // low
      Logger.Log("Arm position: " + arm.getArmPosition());
    }

    if (ps4controller.getShareButtonPressed()) {
      Logger.Log("Share button pressed");
      arm.relax();
    }

    if (ps4controller.getOptionsButtonPressed()) {
      Logger.Log("Options button pressed");
      arm.gethard();
    }

    // the idea here is to run the shooter fo 500ms
    // to get it up to speed, then run the intake for 1000ms
    // then top both of them
    // if (ps4controller.getTouchpadPressed()) {
    //   intake.inComplexAction = true;
    //   Command c =
    //       new SequentialCommandGroup(
    //           new InstantCommand(() -> shooter.shoot()), // run the shooter
    //           new WaitCommand(2),
    //           new InstantCommand(() -> intake.intake()), // run the intake
    //           new WaitCommand(1),
    //           new InstantCommand(
    //               () -> {
    //                 shooter.stop();
    //                 intake.stop();
    //               })); // stop them both
    //   c.schedule(); // make it happen!
    // }

    // if (ps4controller.getPSButtonPressed()) {
    //   intake.inComplexAction = true;
    //   Command c =
    //       new SequentialCommandGroup(
    //           new InstantCommand(() -> intake.intake()),
    //           new InstantCommand(() -> shooter.shoot()),
    //           new WaitCommand(0.5),
    //           new InstantCommand(
    //               () -> {
    //                 intake.stop();
    //                 shooter.stop();
    //               }));
    //   c.schedule();
    // }

    // if (ps4controller.getShareButtonPressed()) {
    //   Command c =
    //       new SequentialCommandGroup(
    //           new InstantCommand(() -> shooter.shoot()),
    //           new WaitCommand(1),
    //           new InstantCommand(
    //               () -> {
    //                 shooter.stop();
    //               }));
    // }

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
