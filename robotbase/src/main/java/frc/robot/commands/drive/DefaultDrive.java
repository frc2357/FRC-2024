package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DefaultDrive extends Command {
  public DefaultDrive() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void execute() {
    Robot.swerve.driveFieldRelative(
        Robot.driverControls.getY(),
        Robot.driverControls.getX(),
        Robot.driverControls.getRotation());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }
}
