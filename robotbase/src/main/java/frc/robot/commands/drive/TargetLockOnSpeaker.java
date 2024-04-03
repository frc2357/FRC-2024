package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CompSwerveTunerConstants;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.util.RobotMath;
import frc.robot.util.Utility;

public class TargetLockOnSpeaker extends Command {
  public int m_startingPipeline;
  public double m_yawOffset;
  public boolean m_stopOnEnd = false;

  public TargetLockOnSpeaker() {
    this(false);
  }

  public TargetLockOnSpeaker(boolean stopOnEnd) {
    m_startingPipeline = Robot.intakeCam.getPipeline();
    addRequirements(Robot.swerve, Robot.shooterCam);
  }

  @Override
  public void initialize() {
    Robot.shooterCam.setAprilTagPipelineActive();
  }

  @Override
  public void execute() {
    var pitch = Robot.shooterCam.getSpeakerTargetPitch();
    if (Double.isNaN(pitch)) {
      Robot.swerve.driveTargetLock(
          Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
          Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
          0,
          0,
          false);
      return;
    }
    int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, pitch);
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
    m_yawOffset = Robot.swerve.updateVisionTargeting(pitch, m_yawOffset);
    Robot.swerve.driveTargetLock(
        Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
        Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
        !Double.isNaN(targetYaw) ? targetYaw : 0,
        m_yawOffset,
        !Double.isNaN(targetYaw));
  }

  @Override
  public boolean isFinished() {
    double yaw = Robot.shooterCam.getSpeakerTargetYaw();
    System.out.println("[TargetLockOnSpeaker] SHOOTER CAM HAS TARGET: " + !Double.isNaN(yaw));
    if (Double.isNaN(yaw) || m_stopOnEnd) {
      return false;
    }
    var isInTolerance = Utility.isWithinTolerance(yaw, m_yawOffset, SWERVE.TARGET_LOCK_TOLERANCE);
    System.out.println(
        "[TargetLockOnSpeaker] SHOOTER CAM HAS TARGET: "
            + !Double.isNaN(yaw)
            + "\n[TargetLockOnSpeaker] In Tolerance: "
            + isInTolerance);
    return isInTolerance;
  }

  @Override
  public void end(boolean interupted) {
    System.out.println("[TargetLockOnSpeaker] WAS INTERRUPTED: " + interupted);
    Robot.swerve.stopMotors();
    Robot.intakeCam.setPipeline(m_startingPipeline);
  }
}
