package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberRotateToRotations extends Command {
  private double m_speed;
  private double m_rotations;

  public ClimberRotateToRotations(double speed, double rotations) {
    m_speed = speed;
    m_rotations = rotations;
    addRequirements(Robot.climber);
  }

  @Override
  public void initialize() {
    Robot.climber.setSpeed(m_speed, m_speed);
  }

  @Override
  public boolean isFinished() {
    return Robot.climber.isAtRotations(m_rotations, m_rotations);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climber.stop();
  }
}
