package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterSetVisionShootingDisabled extends Command {
  public ShooterSetVisionShootingDisabled() {
    addRequirements(Robot.shooter);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stopVisionShooting();
  }
}
