package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TargetLockOnAmp extends Command {
  public TargetLockOnAmp() {
    addRequirements(Robot.swerve, Robot.shooterCam);
  }

  @Override
  public void initialize() {
    Robot.shooterCam.setAprilTagPipelineActive();
  }

  @Override
  public void execute() {
    var target = Robot.shooterCam.getAmpTarget();
    double targetYaw = 0;
    if (target != null) {
      targetYaw = target.getYaw();
    }
    Robot.swerve.driveTargetLock(
        Robot.driverControls.getY(), Robot.driverControls.getX(), targetYaw, target != null);
  }
}
