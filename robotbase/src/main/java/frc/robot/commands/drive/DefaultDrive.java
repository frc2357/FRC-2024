package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class DefaultDrive extends Command {
  public DefaultDrive() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void execute() {
    double x = Robot.driverControls.getX();
    double y = Robot.driverControls.getY();
    double rotation = Robot.driverControls.getRotation();
    
    if (x == 0 && y == 0 && rotation == 0) {
      Robot.swerve.stopMotors();
    } else {
      Robot.swerve.driveFieldRelative(
          y * Constants.SWERVE.MAX_SPEED_METERS_PER_SECOND,
          x * Constants.SWERVE.MAX_SPEED_METERS_PER_SECOND,
          rotation * Constants.SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND);

          //Robot.swerve.driveFieldRelative(0, 0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }
}
