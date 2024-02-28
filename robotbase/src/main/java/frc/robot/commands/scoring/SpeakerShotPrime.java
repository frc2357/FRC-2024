package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SpeakerShotPrime extends Command {
  private double m_topRPMs;
  private double m_bottomRPMs;
  private double m_pivotRotation;

  public SpeakerShotPrime(double topRPMs, double bottomRPMs, double pivotRotation) {
    m_topRPMs = topRPMs;
    m_bottomRPMs = bottomRPMs;
    m_pivotRotation = pivotRotation;
    addRequirements(Robot.shooter, Robot.pivot);
  }

  @Override
  public void initialize() {
    Robot.shooter.setRPMs(m_topRPMs, m_bottomRPMs);
    // Robot.pivot.setPivotRotation(m_pivotRotation);
  }

  @Override
  public boolean isFinished() {
    return Robot.shooter.isAtRPMs(m_topRPMs, m_bottomRPMs) && Robot.pivot.isPivotAtRotation();
  }

  @Override
  public void end(boolean interrupted) {
    // Manually stop shooter and pivot
  }
}
