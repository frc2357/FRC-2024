package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class VelDrive extends Command {

  public VelDrive() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    Robot.swerve.driveFieldRelative(0, 0, 0);
  }

  @Override
  public void execute() {
    System.out.println(Robot.swerve.getModuleStates()[0].speedMetersPerSecond);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }
}
