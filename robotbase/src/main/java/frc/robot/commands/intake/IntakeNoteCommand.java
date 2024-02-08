package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

/*
Command needs to:

1. Run intake at set speed (unknown constant)

2. Complete and stop the intake rollers when
beam-break sensor's beam is broken
*/
public class IntakeNoteCommand extends Command {

  public IntakeNoteCommand() {
    addRequirements(Robot.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Robot.intake.setAxisSpeed(
        Constants.INTAKE.TOP_MOTOR_SPEED_PERCENT_OUTPUT,
        Constants.INTAKE.BOTTOM_MOTOR_SPEED_PERCENT_OUTPUT);
  }

  @Override
  public boolean isFinished() {
    return Robot.intake.getBeamBreakSensor();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
