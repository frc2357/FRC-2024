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
    var targetYaw = Robot.shooterCam.getAmpTargetYaw();

    Robot.swerve.driveTargetLock(
        Robot.driverControls.getY(), Robot.driverControls.getX(), targetYaw != Double.NaN ? targetYaw : 0, targetYaw != Double.NaN);
  }
}
