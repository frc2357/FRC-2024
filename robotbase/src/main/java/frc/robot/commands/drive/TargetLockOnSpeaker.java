package frc.robot.commands.drive;

import org.photonvision.targeting.PhotonTrackedTarget;

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
    PhotonTrackedTarget target = Robot.shooterCam.getSpeakerTarget();
    double targetYaw = target != null ? target.getYaw() : 0;
    Robot.swerve.driveTargetLock(
        Robot.driverControls.getY(), Robot.driverControls.getX(), targetYaw, target != null);
  }

  @Override
  public void end(boolean interupted) {
    Robot.swerve.stopMotors();
  }
}
