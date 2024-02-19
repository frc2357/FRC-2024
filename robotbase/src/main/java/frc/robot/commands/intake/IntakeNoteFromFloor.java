package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.state.RobotState;
import frc.robot.Robot;

public class IntakeNoteFromFloor extends Command {
  public IntakeNoteFromFloor() {
    addRequirements(Robot.intake);
  }

  @Override
  public void execute() {
    Robot.intake.set(
        Robot.intake.isBeamBroken()
            ? INTAKE.TOP_MOTOR_SLOW_PICKUP_SPEED_PERCENT_OUTPUT
            : INTAKE.TOP_MOTOR_PICKUP_SPEED_PERCENT_OUTPUT,
        Robot.intake.isBeamBroken()
            ? INTAKE.BOTTOM_MOTOR_SLOW_PICKUP_SPEED_PERCENT_OUTPUT
            : INTAKE.BOTTOM_MOTOR_PICKUP_SPEED_PERCENT_OUTPUT);
  }

  @Override
  public boolean isFinished() {
    return Robot.intake.hasNotePassedIntake();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
    Robot.intake.resetNotePassedBeamBreak();

    if (!interrupted) {
      Robot.state.setState(RobotState.State.NOTE_STOWED);
    }
  }
}
