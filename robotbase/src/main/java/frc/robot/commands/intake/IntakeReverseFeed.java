package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.Robot;

public class IntakeReverseFeed extends Command {

  public IntakeReverseFeed() {

    addRequirements(Robot.intake);
  }

  @Override
  public void initialize() {
    Robot.intake.set(
        -INTAKE.TOP_MOTOR_SOURCE_INTAKE_SPEED_PERCENT_OUTPUT,
        -INTAKE.BOTTOM_MOTOR_SOURCE_INTAKE_SPEED_PERCENT_OUTPUT);
  }

  @Override
  public boolean isFinished() {
    return Robot.intake.isBeamBroken();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
    /*
    if (!interrupted) {
      Robot.state.setState(RobotState.State.NOTE_STOWED);
     */
  }
}
