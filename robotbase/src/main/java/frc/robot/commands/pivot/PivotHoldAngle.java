package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class PivotHoldAngle extends Command {
  private double m_angle;
  private boolean m_stopOnEnd;

  public PivotHoldAngle(double angle) {
    this(angle, true);
  }

  public PivotHoldAngle(double angle, boolean stopOnEnd) {
    m_angle = angle;
    m_stopOnEnd = stopOnEnd;
    addRequirements(Robot.pivot);
  }

  @Override
  public void initialize() {
    Robot.pivot.setAngle(m_angle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (m_stopOnEnd) {
      Robot.pivot.stop();
    }
  }
}
