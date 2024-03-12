package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

// The target lock command but ensures no stick input is possible
public class AutoTargetLockOnSpeaker extends Command {
  public int m_startingPipeline;

  public AutoTargetLockOnSpeaker() {
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

    Robot.swerve.driveTargetLock(0, 0, !Double.isNaN(targetYaw) ? targetYaw : 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interupted) {
    Robot.swerve.stopMotors();
    Robot.intakeCam.setPipeline(m_startingPipeline);
  }
}
