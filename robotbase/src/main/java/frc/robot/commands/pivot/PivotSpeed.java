package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class PivotSpeed extends Command {
  private double m_speed;

  public PivotSpeed(double speed) {
    m_speed = speed;
    addRequirements(Robot.pivot);
  }

  @Override
  public void execute() {
    Robot.pivot.setSpeed(m_speed);
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
