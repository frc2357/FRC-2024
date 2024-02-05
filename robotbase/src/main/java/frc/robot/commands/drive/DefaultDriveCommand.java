package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class DefaultDriveCommand extends Command {
  public DefaultDriveCommand() {
    addRequirements(Robot.drive);
  }

  @Override
  public void execute() {
    Robot.drive.drive(
        Robot.driverControls.getY() * Constants.SWERVE.MAX_SPEED_METERS_PER_SECOND,
        Robot.driverControls.getX() * Constants.SWERVE.MAX_SPEED_METERS_PER_SECOND,
        Robot.driverControls.getRotation()
            * Constants.SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.drive.drive(0, 0, 0);
  }
}
