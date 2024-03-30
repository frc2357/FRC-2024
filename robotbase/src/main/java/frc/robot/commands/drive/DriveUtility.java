package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.APRIL_TAG_IDS;
import frc.robot.Constants.SHOOTER_PHOTON_CAMERA;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
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
      return Math.copySign(yawTolerance * SWERVE.VISION_CLOSE_YAW_FACTOR, yawTolerance);
    }
    return yawTolerance;
  }

  public static double calculateRotationError(double rotationError, double rotationSetpoint) {
    if (Utility.isWithinTolerance(
        rotationError, rotationSetpoint, SWERVE.VISION_ROTATION_TOLERANCE_RADIANS)) {
      return rotationSetpoint;
    }
    return rotationError;
  }

  public static double getAmpRotationGoal() {
    boolean redAmpValid =
        Robot.shooterCam.isValidTarget(
            APRIL_TAG_IDS.RED_AMP, SHOOTER_PHOTON_CAMERA.AMP_TARGET_TIMEOUT_MS);
    boolean blueAmpValid =
        Robot.shooterCam.isValidTarget(
            APRIL_TAG_IDS.BLUE_AMP, SHOOTER_PHOTON_CAMERA.AMP_TARGET_TIMEOUT_MS);

    if (redAmpValid && blueAmpValid) {
      DriverStation.reportError("How in the world do you see both amps at the same time", false);
      return Double.NaN;
    } else if (blueAmpValid) {
      return SWERVE.BLUE_AMP_ROTATION_SETPOINT_RADIANS;
    } else if (redAmpValid) {
      return SWERVE.RED_AMP_ROTATION_SETPOINT_RADIANS;
    } else {
      System.err.println("No Amp target");
      return Double.NaN;
    }
  }

  public static double getStageRotationGoal() {
    int bestTagId = Robot.shooterCam.getBestTargetFiducialId();

    switch (bestTagId) {
      case APRIL_TAG_IDS.BLUE_STAGE_LEFT:
        return SWERVE.BLUE_LEFT_STAGE_ROTATION_SETPOINT_RADIANS;
      case APRIL_TAG_IDS.BLUE_STAGE_RIGHT:
        return SWERVE.BLUE_RIGHT_STAGE_ROTATION_SETPOINT_RADIANS;
      case APRIL_TAG_IDS.BLUE_STAGE_MIDDLE:
        return SWERVE.BLUE_CENTER_STAGE_ROTATION_SETPOINT_RADIANS;
      case APRIL_TAG_IDS.RED_STAGE_LEFT:
        return SWERVE.RED_LEFT_STAGE_ROTATION_SETPOINT_RADIANS;
      case APRIL_TAG_IDS.RED_STAGE_RIGHT:
        return SWERVE.RED_RIGHT_STAGE_ROTATION_SETPOINT_RADIANS;
      case APRIL_TAG_IDS.RED_STAGE_MIDDLE:
        return SWERVE.RED_CENTER_STAGE_ROTATION_SETPOINT_RADIANS;
      default:
        return Double.NaN;
    }
  }
}
