package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterPivotIntakeFromSource extends Command {
  public ShooterPivotIntakeFromSource() {
    addRequirements(Robot.pivot);
  }

  @Override
  public void initialize() {
    Robot.pivot.setPivotRotations(Constants.SHOOTER_PIVOT.INTAKE_FROM_SOURCE_ROTATIONS);
  }

  @Override
  public boolean isFinished() {
    return Robot.pivot.isPivotAtRotations();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.pivot.stop();
  }
}
