package frc.robot.commands.endAffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Robot;

public class EndAffectorPreloadNote extends Command {
  private boolean m_hasSeenTopEdge = false;
  private boolean m_hasPassedTopEdge = false;
  private boolean m_isAtTop = false;
  private boolean m_isDone = false;

  public EndAffectorPreloadNote() {
    addRequirements(Robot.endAffector);
  }

  @Override
  public void initialize() {
    Robot.endAffector.setSpeed(END_AFFECTOR.INTAKE_SPEED);
  }

  @Override
  public void execute() {
    if (!m_hasSeenTopEdge && Robot.endAffector.getProximitySensor()) {
      m_hasSeenTopEdge = true;
    } else if (m_hasSeenTopEdge && !m_hasPassedTopEdge && !Robot.endAffector.getProximitySensor()) {
      m_hasPassedTopEdge = true;
      // Robot.endAffector.setSpeed(-0.5);
    } else if (m_hasPassedTopEdge && !m_isAtTop && Robot.endAffector.getProximitySensor()) {
      m_isAtTop = true;
      Robot.endAffector.setSpeed(0.5);
    } else if (m_isAtTop && !m_isDone && !Robot.endAffector.getProximitySensor()) {
      m_isDone = true;
    }
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.endAffector.setSpeed(0);
    Robot.endAffector.setProximitySensorPower(false);
  }
}
