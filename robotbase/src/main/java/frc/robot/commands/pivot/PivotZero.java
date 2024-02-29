package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class PivotZero extends ParallelDeadlineGroup {
  public static class UntilPivotZero extends Command {
    @Override
    public boolean isFinished() {
      return Math.abs(Robot.pivot.getVelocity()) < Constants.PIVOT.ZERO_SPEED_STOP_TOLERANCE;
    }
  }

  public PivotZero() {
    super(
        // First wait until we start moving, then wait until the velocity stops
        new SequentialCommandGroup(
                new WaitCommand(Constants.PIVOT.ZERO_SPEED_INITIAL_SECONDS), new UntilPivotZero())
            .finallyDo(
                (boolean interrupted) -> {
                  if (!interrupted) {
                    Robot.pivot.setZero();
                  } else {
                    System.err.println("[Pivot] Zero interrupted!");
                  }
                }),

        // Pivot along the axis until the above finishes
        new PivotAxis(
            new AxisInterface() {
              @Override
              public double getValue() {
                return Constants.PIVOT.ZERO_SPEED;
              }
            }));
  }
}
;
