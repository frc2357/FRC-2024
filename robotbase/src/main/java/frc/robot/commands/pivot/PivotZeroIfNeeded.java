package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class PivotZeroIfNeeded extends ConditionalCommand {
  public PivotZeroIfNeeded() {
    super(new WaitCommand(0), new PivotZero(), () -> Robot.pivot.isZeroed());
  }
}
