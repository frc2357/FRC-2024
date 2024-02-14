package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterSetVisionShootingEnabled extends Command {
  private boolean m_isVisionShooting;

  public ShooterSetVisionShootingEnabled(boolean m_isVisionShooting) {
    m_isVisionShooting = m_isVisionShooting;
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.setVisionShootingEnabled(m_isVisionShooting);
  }
}
