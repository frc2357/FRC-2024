package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DriveDistance extends Command {
  private double m_xDistance;
  private double m_yDistance;
  private double m_timeSeconds;
  private long m_startTimeMillis;

  public DriveDistance(double xDistance, double yDistance, double timeSeconds) {
    m_xDistance = xDistance;
    m_yDistance = yDistance;
    m_timeSeconds = timeSeconds;
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    m_startTimeMillis = System.currentTimeMillis();
    Robot.swerve.drive(m_xDistance / m_timeSeconds, m_yDistance / m_timeSeconds, 0);
  }

  @Override
  public boolean isFinished() {
    return (m_timeSeconds != -1)
        && (System.currentTimeMillis() >= m_startTimeMillis + (1000 * m_timeSeconds));
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.drive(0, 0, 0);
  }
}
