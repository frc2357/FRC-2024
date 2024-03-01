package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AutoPivotStop extends Command {
  public AutoPivotStop() {
    addRequirements(Robot.pivot);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.pivot.stop();
  }
}
