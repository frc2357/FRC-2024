package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeRunUntilBeamState extends Command {
  private double m_speed;
  private boolean m_desiredBeamState;

  public IntakeRunUntilBeamState(double speed, boolean desiredBeamState) {
    m_speed = speed;
    m_desiredBeamState = desiredBeamState;
    addRequirements(Robot.intake);
  }

  @Override
  public void initialize() {
    Robot.intake.set(m_speed);
  }

  @Override
  public boolean isFinished() {
    return Robot.intake.isBeamBroken() == m_desiredBeamState;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
