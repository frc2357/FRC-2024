package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class DefaultPivot extends Command {
  public DefaultPivot() {
    addRequirements(Robot.pivot);
  }

  @Override
  public void initialize() {
    Robot.pivot.setPivotRotation(Constants.PIVOT.DEFAULT_PIVOT_ROTATION);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
