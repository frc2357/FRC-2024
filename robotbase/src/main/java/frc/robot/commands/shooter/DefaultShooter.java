package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DefaultShooter extends Command {
  public DefaultShooter() {
    addRequirements(Robot.shooter);
  }

  @Override
  public void execute() {
    Robot.shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
