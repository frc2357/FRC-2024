package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.util.RobotMath;
import frc.robot.util.Utility;

public class AutoWaitForTargeting extends Command {
  public AutoWaitForTargeting() {}

  // Can remove getYaw and move to setting target yaw in drive. Doing this here just because I
  // do not want to modify activley working code at comp
  @Override
  public boolean isFinished() {
    var pitch = Robot.shooterCam.getSpeakerTargetPitch();
    if (Double.isNaN(pitch)) {
      return true;
    }
    int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, pitch);
    if (curveIndex == -1) {
      curveIndex = 0;
    }
    var targetYaw = Robot.shooterCam.getSpeakerTargetYaw();
    var yawSetpoint = Robot.shooterCurve[curveIndex][3];
    return Robot.shooter.isAtTargetSpeed()
        && Robot.pivot.isPivotAtAngle()
        && !Double.isNaN(targetYaw)
        && Utility.isWithinTolerance(targetYaw, yawSetpoint, SWERVE.AUTO_TARGET_LOCK_YAW_TOLERANCE);
  }
}
