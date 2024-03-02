package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AutoShooterSetRPMAndFinish extends Command {
  private double m_RPM;

  public AutoShooterSetRPMAndFinish(double RPM) {
    m_RPM = RPM;
    addRequirements(Robot.shooter);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.setRPM(m_RPM);
    // Stop manually in auto
  }
}
