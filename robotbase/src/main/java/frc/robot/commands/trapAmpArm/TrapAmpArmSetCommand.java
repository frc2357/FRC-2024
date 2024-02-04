package frc.robot.commands.trapAmpArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TrapAmpArmSetCommand extends Command {
  private double speed;

  public TrapAmpArmSetCommand(double speed) {
    this.speed = speed;
    addRequirements(Robot.trapAmpArm);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interupted) {
    Robot.trapAmpArm.set(speed);
  }
}
