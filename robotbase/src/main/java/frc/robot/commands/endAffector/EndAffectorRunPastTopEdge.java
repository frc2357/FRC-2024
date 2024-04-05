package frc.robot.commands.endAffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Robot;

public class EndAffectorRunPastTopEdge extends Command {
  private boolean m_hasSeenTopEdge;
  private boolean m_hasPassedTopEdge;
  private boolean m_turnOfProxOnEnd;

  public EndAffectorRunPastTopEdge(boolean turnOfProxOnEnd) {
    addRequirements(Robot.endAffector);
    m_turnOfProxOnEnd = turnOfProxOnEnd;
  }

  @Override
  public void initialize() {
    m_hasSeenTopEdge = false;
    m_hasPassedTopEdge = false;
    Robot.endAffector.setSpeed(END_AFFECTOR.INTAKE_SPEED);
  }

  @Override
  public void execute() {
    if (!m_hasSeenTopEdge && Robot.endAffector.getProximitySensor()) {
      m_hasSeenTopEdge = true;
    } else if (m_hasSeenTopEdge && !m_hasPassedTopEdge && !Robot.endAffector.getProximitySensor()) {
      m_hasPassedTopEdge = true;
    }
  }

  @Override
  public boolean isFinished() {
    return m_hasPassedTopEdge;
  }

  @Override
  public void end(boolean interrupted) {
    if (m_turnOfProxOnEnd) {
      Robot.endAffector.setProximitySensorPower(false);
    }
  }
}
