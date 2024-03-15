package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIVOT;
import frc.robot.Robot;

public class PivotHoldAngle extends Command {
  private double m_angle;
  private boolean m_stopOnEnd;
  private double m_pivotOffsetAngle;
  private boolean m_includeAngleOffset;

  public PivotHoldAngle(double angle) {
    this(angle, true, false);
  }

  public PivotHoldAngle(double angle, boolean stopOnEnd, boolean includeAngleOffset) {
    m_angle = angle;
    m_stopOnEnd = stopOnEnd;
    m_includeAngleOffset = includeAngleOffset;
    addRequirements(Robot.pivot);
  }

  @Override
  public void initialize() {
    m_pivotOffsetAngle =
        m_includeAngleOffset ? SmartDashboard.getNumber(PIVOT.PIVOT_OFFSET_KEY, 0.0) : 0;
    Robot.pivot.setAngle(m_angle + m_pivotOffsetAngle);
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
