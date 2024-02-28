package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.pivot.PivotZeroIfNeeded;

public class ZeroAllIfNeeded extends ParallelCommandGroup {
  public ZeroAllIfNeeded() {
    super(new PivotZeroIfNeeded());
  }
}
