package frc.robot.commands.endAffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class EndAffectorRunToTop extends Command {
  private boolean m_isAtTop;
  private boolean m_isDone;
  private Timer m_timer;

  public EndAffectorRunToTop() {
    addRequirements(Robot.endAffector);
    m_timer = new Timer();
  }

  @Override
  public void initialize() {
    m_isAtTop = false;
    m_isDone = false;
    m_timer.reset();
    Robot.endAffector.setSpeed(-0.5);
  }

  @Override
  public void execute() {
    if (!m_isAtTop && Robot.endAffector.getProximitySensor()) {
      m_isAtTop = true;
      Robot.endAffector.setSpeed(0.5);
      System.out.println("isAtTop");
    } else if (!m_isDone && m_isAtTop && !Robot.endAffector.getProximitySensor()) {
      m_isDone = true;
      m_timer.start();
      System.out.println("isDone");
    }
  }

  @Override
  public boolean isFinished() {
    return m_isDone && m_timer.hasElapsed(0.1);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("end: " + interrupted);
    Robot.endAffector.stop();
    Robot.endAffector.setProximitySensorPower(false);
  }
}
