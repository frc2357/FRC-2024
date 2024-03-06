package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DriveAtSpeed extends Command {
  private double m_xSpeed;
  private double m_ySpeed;
  private double m_timeSeconds;
  private Timer m_timer;

  public DriveAtSpeed(double xSpeed, double ySpeed, double timeSeconds) {
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_timeSeconds = timeSeconds;
    m_timer = new Timer();
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    Robot.swerve.driveRobotRelative(m_xSpeed, m_ySpeed, 0);
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_timeSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }
}
