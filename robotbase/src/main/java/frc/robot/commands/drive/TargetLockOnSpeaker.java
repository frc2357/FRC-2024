package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CompSwerveTunerConstants;
import frc.robot.Robot;

public class TargetLockOnSpeaker extends Command {
  public int m_startingPipeline;

  public TargetLockOnSpeaker() {
    m_startingPipeline = Robot.intakeCam.getPipeline();
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
        Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
        Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
        !Double.isNaN(targetYaw) ? targetYaw : 0,
        !Double.isNaN(targetYaw));
  }

  @Override
  public void end(boolean interupted) {
    Robot.swerve.stopMotors();
    Robot.intakeCam.setPipeline(m_startingPipeline);
  }
}
