package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CompSwerveTunerConstants;
import frc.robot.Robot;
import frc.robot.util.RobotMath;

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
    int curveIndex =
        RobotMath.getCurveSegmentIndex(
            Robot.shooterCurve, Robot.shooterCam.getSpeakerTargetPitch());
    if (curveIndex == -1) {
      Robot.swerve.driveTargetLock(
          Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
          Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
          0,
          0,
          false);
      return;
    }
    var targetYaw = Robot.shooterCam.getSpeakerTargetYaw();
    double yawOffset = Robot.shooterCurve[curveIndex][3];
    Robot.swerve.driveTargetLock(
        Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
        Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
        !Double.isNaN(targetYaw) ? targetYaw : 0,
        yawOffset,
        !Double.isNaN(targetYaw));
  }

  @Override
  public void end(boolean interupted) {
    Robot.swerve.stopMotors();
    Robot.intakeCam.setPipeline(m_startingPipeline);
  }
}
