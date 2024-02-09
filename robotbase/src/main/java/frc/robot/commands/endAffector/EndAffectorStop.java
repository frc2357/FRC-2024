package frc.robot.commands.endAffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class EndAffectorStop extends Command {
  public EndAffectorStop() {
    addRequirements(Robot.endAffector);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.endAffector.stop();
  }
}
