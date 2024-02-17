package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterPivotSetRotation extends Command {
  private double m_rotation;

  public ShooterPivotSetRotation(double rotation) {
    m_rotation = rotation;
    addRequirements(Robot.pivot);
  }

  @Override
  public void initialize() {
    Robot.pivot.setPivotRotation(m_rotation);
  }

  @Override
  public boolean isFinished() {
    return Robot.pivot.isPivotAtRotation();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.pivot.stop();
  }
}
