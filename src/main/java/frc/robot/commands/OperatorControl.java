package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Logger;
import frc.robot.Settings;
import frc.robot.subsystems.Arm;

public class OperatorControl extends Command {

  private PS4Controller ps4controller;

  private Arm arm;

  public OperatorControl(Arm arm) {
    this.arm = arm;

    ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);
  }

  @Override
  public void execute() {

    if (ps4controller.getTriangleButton()) {
      Logger.Log("Triangle button pressed");
      arm.setArmPositionRotations(-0.025); // was AMPSHOOTING
    }

    if (ps4controller.getSquareButtonPressed()) {
      Logger.Log("Square button pressed");
      double position = arm.getArmPosition();
      Logger.Log("Arm position: " + position);
    }

    if (ps4controller.getShareButtonPressed()) {
      Logger.Log("Share button pressed");
      arm.relax();
    }

    if (ps4controller.getOptionsButtonPressed()) {
      Logger.Log("Options button pressed");
      arm.gethard();
      // shooter.gethard();
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
