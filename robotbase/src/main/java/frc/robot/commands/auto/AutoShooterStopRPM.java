package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AutoShooterStopRPM extends Command {
  public AutoShooterStopRPM() {
    addRequirements(Robot.shooter);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stop();
  }
}
