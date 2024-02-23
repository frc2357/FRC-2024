package frc.robot.commands.autoClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CLIMBER;
import frc.robot.Robot;

public class WaitForEndAffectorDrop extends Command {
  private boolean m_hasPassedSetpoint;

  @Override
  public boolean isFinished() {
    double pitch = Robot.swerve.getPitch();

    if (!m_hasPassedSetpoint && pitch >= CLIMBER.LEVEL_CLIMB_PITCH_SETPOINT) {
      m_hasPassedSetpoint = true;
    } else if (m_hasPassedSetpoint && pitch <= CLIMBER.LEVEL_CLIMB_PITCH_SETPOINT) {
      return true;
    }
    return false;
  }
}
