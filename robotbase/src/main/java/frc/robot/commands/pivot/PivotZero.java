package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class PivotZero extends Command {
  public boolean isZeroed() {
    return Robot.pivot.getPivotVelocity() <= Constants.SHOOTER.ZERO_SPEED_STOP_TOLERANCE;
  }

  @Override
  public void initialize() {
    Robot.pivot.setPivotAxisSpeed(Constants.SHOOTER.ZERO_SPEED);
  }

  @Override
  public boolean isFinished() {
    return isZeroed();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.pivot.stop();

    if (!interrupted) {
      Robot.pivot.zero();
    } else {
      DriverStation.reportError("Pivot Zero interrupted!", false);
    }
  }
}
