package frc.robot.commands.extensionArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class ExtensionArmZero extends ParallelDeadlineGroup {
  public static class UntilArmZero extends Command {
    @Override
    public boolean isFinished() {
      return Math.abs(Robot.extensionArm.getVelocity())
          < Constants.EXTENSION_ARM.ZERO_SPEED_STOP_TOLERANCE;
    }
  }

  public ExtensionArmZero() {
    super(
        // First wait until we start moving, then wait until the velocity stops
        new SequentialCommandGroup(
                new WaitCommand(Constants.EXTENSION_ARM.ZERO_SPEED_INITIAL_SECONDS),
                new UntilArmZero())
            .finallyDo(
                (boolean interrupted) -> {
                  if (!interrupted) {
                    Robot.extensionArm.setZero();
                  } else {
                    System.err.println("[Extension Arm] Zero interrupted!");
                  }
                }),

        // Retracts until the above finishes
        new ExtensionArmSpeed(Constants.EXTENSION_ARM.ZERO_SPEED));
  }
}
