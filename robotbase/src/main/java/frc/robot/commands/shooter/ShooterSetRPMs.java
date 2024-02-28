package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterSetRPMs extends Command {
  private double m_topRpms;
  private double m_bottomRpms;

  public ShooterSetRPMs(double top, double bottom) {
    m_topRpms = top;
    m_bottomRpms = bottom;
    addRequirements(Robot.shooter);
  }

  @Override
  public void initialize() {
    Robot.shooter.setRPMs(m_topRpms, m_bottomRpms);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop rollers manually
  }
}
