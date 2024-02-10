package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeNoteFromSource extends Command {

  public IntakeNoteFromSource() {
    addRequirements(Robot.intake);
  }

  @Override
  public void initialize() {
    Robot.intake.set(
        -Constants.INTAKE.TOP_MOTOR_EAT_SPEED_PERCENT_OUTPUT,
        -Constants.INTAKE.BOTTOM_MOTOR_EAT_SPEED_PERCENT_OUTPUT);
  }

  @Override
  public boolean isFinished() {
    return Robot.intake.isBeamBroken();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
