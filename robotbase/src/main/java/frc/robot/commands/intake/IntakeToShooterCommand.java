package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeToShooterCommand extends Command {

  public IntakeToShooterCommand() {
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
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
