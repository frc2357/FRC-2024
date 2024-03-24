package frc.robot.commands.endAffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Robot;

public class EndAffectorPreloadNote extends Command {
  private boolean m_hasSeenTopEdge = false;
  private boolean m_hasPassedTopEdge = false;
  private boolean m_isAtTop = false;

  public EndAffectorPreloadNote() {
    addRequirements(Robot.endAffector);
  }

  @Override
  public void initialize() {
    Robot.endAffector.setProximitySensorPower(true);
    Robot.endAffector.setSpeed(END_AFFECTOR.INTAKE_SPEED);
  }

  @Override
  public void execute() {
    if (!m_hasSeenTopEdge && Robot.endAffector.getProximitySensor()) {
      System.out.println("hasSeen");
      m_hasSeenTopEdge = true;
    } else if (m_hasSeenTopEdge && !m_hasPassedTopEdge && !Robot.endAffector.getProximitySensor()) {
      System.out.println("hasPassed");
      m_hasPassedTopEdge = true;
    } else if (m_hasPassedTopEdge && !m_isAtTop && Robot.endAffector.getProximitySensor()) {
      System.out.println("atTop");
      m_isAtTop = true;
    }
  }

  @Override
  public boolean isFinished() {
    return m_isAtTop;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.endAffector.setSpeed(0);
    Robot.endAffector.setProximitySensorPower(false);
  }
}
