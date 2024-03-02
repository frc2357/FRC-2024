package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterSetRPM extends Command {
  private double m_RPM;
  private boolean m_stopOnEnd;

  public ShooterSetRPM(double RPM) {
    this(RPM, true);
  }

  public ShooterSetRPM(double RPM, boolean stopOnEnd) {
    m_RPM = RPM;
    m_stopOnEnd = stopOnEnd;
    addRequirements(Robot.shooter);
  }

  @Override
  public void initialize() {
    Robot.shooter.setRPM(m_RPM);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (m_stopOnEnd) {
      Robot.shooter.stop();
    }
  }
}
