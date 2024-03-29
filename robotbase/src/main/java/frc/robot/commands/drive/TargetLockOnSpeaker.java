package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CompSwerveTunerConstants;
import frc.robot.Robot;
import frc.robot.util.RobotMath;

public class TargetLockOnSpeaker extends Command {
  public int m_startingPipeline;
  public double m_yawOffset;

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
    updateVisionTargeting(pitch);
    Robot.swerve.driveTargetLock(
        Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
        Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
        !Double.isNaN(targetYaw) ? targetYaw : 0,
        m_yawOffset,
        !Double.isNaN(targetYaw));
  }

  @Override
  public void end(boolean interupted) {
    Robot.swerve.stopMotors();
    Robot.intakeCam.setPipeline(m_startingPipeline);
  }

  private void updateVisionTargeting(double pitch) {
    int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, pitch);
    if (curveIndex == -1) {
      return;
    }

    double[] high = Robot.shooterCurve[curveIndex];
    double[] low = Robot.shooterCurve[curveIndex + 1];

    double highPitch = high[0];
    double lowPitch = low[0];
    double highYawSetopint = high[3];
    double lowYawSetpoint = low[3];

    m_yawOffset =
        RobotMath.linearlyInterpolate(
                highYawSetopint, lowYawSetpoint, highPitch, lowPitch, pitch);
  }
}
