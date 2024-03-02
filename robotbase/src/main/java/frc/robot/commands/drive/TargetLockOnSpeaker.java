package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TargetLockOnSpeaker extends Command {
  public TargetLockOnSpeaker() {
    addRequirements(Robot.swerve, Robot.shooterCam);
  }

  @Override
  public void initialize() {
    Robot.shooterCam.setAprilTagPipelineActive();
  }

  @Override
  public void execute() {
    var targetYaw = Robot.shooterCam.getSpeakerTargetYaw();

    Robot.swerve.driveTargetLock(
        Robot.driverControls.getY(), Robot.driverControls.getX(), targetYaw != Double.NaN ? targetYaw : 0, targetYaw != Double.NaN);
  }

  @Override
  public void end(boolean interupted) {
    Robot.swerve.stopMotors();
  }
}
