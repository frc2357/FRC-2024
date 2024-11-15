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
    m_startingPipeline = Robot.shooterCam.getPipeline();
    addRequirements(Robot.swerve, Robot.shooterCam);
  }

  @Override
  public void initialize() {
    Robot.shooterCam.setAprilTagPipelineActive();
  }

  @Override
  public void execute() {
    var pitch = Robot.shooterCam.getSpeakerTargetPitch();
    if (Double.isNaN(pitch)) { // stops rotating if we dont have valid target data.
      Robot.swerve.driveTargetLock(
          Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
          Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
          0,
          0,
          false);
      return;
    }
    int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, pitch);
    if (curveIndex == -1) { // stops rotating if we dont have a valid curve index.
      Robot.swerve.driveTargetLock(
          Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
          Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
          0,
          0,
          false);
      return;
    }
    // dont need to check for NaN as we already have.
    var targetYaw = Robot.shooterCam.getSpeakerTargetYaw();
    m_yawOffset = Robot.swerve.updateVisionTargeting(pitch, m_yawOffset);
    Robot.swerve.driveTargetLock(
        Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
        Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
        !Double.isNaN(targetYaw) ? targetYaw : 0,
        m_yawOffset,
        !Double.isNaN(targetYaw));
  }

  @Override
  public boolean isFinished() {
    double yaw = Robot.shooterCam.getSpeakerTargetYaw();
    if (Double.isNaN(yaw) || m_stopOnEnd) {
      return false;
    }
    var isInTolerance = Utility.isWithinTolerance(yaw, m_yawOffset, SWERVE.TARGET_LOCK_TOLERANCE);
    return isInTolerance;
  }

  @Override
  public void end(boolean interupted) {
    System.out.println("[TargetLockOnSpeaker] WAS INTERRUPTED: " + interupted);
    Robot.swerve.stopMotors();
    Robot.shooterCam.setPipeline(m_startingPipeline);
  } 
}
