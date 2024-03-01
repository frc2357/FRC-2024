package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberSpeed extends Command {
  private double m_leftSpeed;
  private double m_rightSpeed;

  public ClimberSpeed(double leftSpeed, double rightSpeed) {
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
    addRequirements(Robot.climber);
  }

  @Override
  public void execute() {
    Robot.climber.setSpeed(m_leftSpeed, m_rightSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climber.stop();
  }
}
