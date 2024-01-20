package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberStopCommand extends Command {
  public ClimberStopCommand() {
    addRequirements(Robot.climber);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climber.stop();
  }
}
