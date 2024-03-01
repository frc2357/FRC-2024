package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AutoPivotSetAngle extends Command {
  private double m_angle;

  public AutoPivotSetAngle(double angle) {
    m_angle = angle;
    addRequirements(Robot.pivot);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop manually at end of auto
    Robot.pivot.setAngle(m_angle);
  }
}
