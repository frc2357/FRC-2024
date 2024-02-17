package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class DefaultShooterPivot extends Command {
  public DefaultShooterPivot() {
    addRequirements(Robot.pivot);
  }

  @Override
  public void initialize() {
    Robot.pivot.setPivotRotation(Constants.SHOOTER_PIVOT.DEFAULT_SHOOTER_PIVOT_ROTATION);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
