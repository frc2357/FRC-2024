package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeRun extends Command {
  private double m_speed;
  private boolean m_stopOnEnd;

  public IntakeRun(double speed) {
    this(speed, true);
  }

  public IntakeRun(double speed, boolean stopOnEnd) {
    m_speed = speed;
    m_stopOnEnd = stopOnEnd;
    addRequirements(Robot.intake);
  }

  @Override
  public void initialize() {
    Robot.intake.set(m_speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (m_stopOnEnd) {
      Robot.intake.stop();
    }
  }
}
