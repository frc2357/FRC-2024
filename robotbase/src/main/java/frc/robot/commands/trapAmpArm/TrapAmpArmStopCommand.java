package frc.robot.commands.trapAmpArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TrapAmpArmStopCommand extends Command {
  public TrapAmpArmStopCommand() {
    addRequirements(Robot.trapAmpArm);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interupted) {
    Robot.trapAmpArm.stop();
  }
}
