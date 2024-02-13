package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetShooterPivotVisionShooting extends Command {
  private boolean m_isVisionShooting;

  public SetShooterPivotVisionShooting(boolean isVisionShooting) {
    m_isVisionShooting = isVisionShooting;
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.pivot.setVisionShootingEnabled(m_isVisionShooting);
  }
}
