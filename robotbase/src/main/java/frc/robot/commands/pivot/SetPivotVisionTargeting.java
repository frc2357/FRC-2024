package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetPivotVisionTargeting extends Command {
  private boolean m_isVisionTargeting;

  public SetPivotVisionTargeting(boolean isVisionTargeting) {
    m_isVisionTargeting = isVisionTargeting;
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    if (m_isVisionTargeting) {
      Robot.pivot.startVisionShooting();
    } else {
      Robot.pivot.stopVisionShooting();
    }
  }
}
