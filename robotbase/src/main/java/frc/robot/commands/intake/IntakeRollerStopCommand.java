package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeRollerStopCommand extends Command {

  public IntakeRollerStopCommand() {
    addRequirements(Robot.intake);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
