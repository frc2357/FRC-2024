package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SWERVE;
import frc.robot.util.Utility;

public class DriveUtility {

  public static double adjustPitchForApriltag(
      double pitch, double pitchOffset, double pitchTolerance) {
    if (Utility.isWithinTolerance(pitch, pitchOffset, pitchTolerance)) {
      return pitchOffset;
    }
    return pitch;
  }

  public static double adjustYawForApriltag(
      double yaw,
      double pitch,
      double rotationError,
      double pitchOffset,
      double closePitchThreshold,
      double yawTolerance) {
    yawTolerance =
        calculateYawToleranceForApriltag(pitch, pitchOffset, closePitchThreshold, yawTolerance);

    // Reduce yaw based on how far off our rotation is so the x controller doesn't
    // over compensate
    yaw -= Rotation2d.fromRadians(rotationError).getDegrees();
    if (Utility.isWithinTolerance(yaw, 0, yawTolerance)) {
      return 0;
    }
    return yaw;
  }

  public static double calculateYawToleranceForApriltag(
      double pitch, double pitchOffset, double closePitchThreshold, double yawTolerance) {
    // Increase yaw tolerance when close to target since yaw is more sensitive at
    // shorter distances
    if (Utility.isWithinTolerance(pitch, pitchOffset, closePitchThreshold)) {
      return Math.copySign(yawTolerance * SWERVE.APRILTAG_CLOSE_YAW_FACTOR, yawTolerance);
    }
    return yawTolerance;
  }

  public static double calculateRotationError(double rotationError) {
    if (Utility.isWithinTolerance(rotationError, 0, SWERVE.APRILTAG_ROTATION_TOLERANCE)) {
      return 0;
    }
    return rotationError;
  }
}
