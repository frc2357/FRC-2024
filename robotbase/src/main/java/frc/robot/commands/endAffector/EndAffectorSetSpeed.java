package frc.robot.commands.endAffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class EndAffectorSetSpeed extends Command {
  private double m_speed;

  public EndAffectorSetSpeed(double speed) {
    m_speed = speed;
    addRequirements(Robot.endAffector);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.endAffector.setSpeed(m_speed);
  }
}
