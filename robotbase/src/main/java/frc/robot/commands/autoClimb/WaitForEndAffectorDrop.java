package frc.robot.commands.autoClimb;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitForEndAffectorDrop extends Command {
  private double m_prevPitch;
  private double m_currentPitch;

  @Override
  public boolean isFinished() {
    return true;
    // m_currentPitch = Robot.swerve.getPitch();
    // double difference = m_currentPitch - m_prevPitch;
    // m_prevPitch = m_currentPitch;

    // return difference >= Constants.CLIMBER.END_AFFECTOR_DROP_DELTA_PITCH;
  }
}
