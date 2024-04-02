package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CompSwerveTunerConstants;
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
          y * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
          x * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
          rotation * Constants.SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Robot.swerve.stopMotors();
  }
}
