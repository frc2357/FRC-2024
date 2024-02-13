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
    Robot.pivot.setPivotSetpoint(Constants.SHOOTER_PIVOT.DEFAULT_SHOOTER_PIVOT_SETPOINT);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.pivot.stop();
  }
}
