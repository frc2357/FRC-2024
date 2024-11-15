package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.util.RobotMath;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

  private final SwerveRequest.ApplyRobotSpeeds robotSpeedRequest =
      new SwerveRequest.ApplyRobotSpeeds();

  private final SwerveRequest.FieldCentric fieldRelative =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.RobotCentric robotRelative =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    this(driveTrainConstants, 0, modules);
  }

  public void applyRequest(Supplier<SwerveRequest> requestSupplier) {
    setControl(requestSupplier.get());
  }

  public double getYaw() {
    return getPigeon2().getYaw().getValueAsDouble();
  }

  // Pigeon is rotated 90 degrees so pitch and roll are flipped
  public double getRoll() {
    return getPigeon2().getPitch().getValueAsDouble();
  }

  // Pigeon is rotated 90 degrees so pitch and roll are flipped
  public double getPitch() {
    return getPigeon2().getRoll().getValueAsDouble();
  }

  /**
   * The method to use to drive while targetlocking.
   *
   * @param velocityXSpeedMetersPerSecond The desired speed on the X axis in meters per second.
   * @param velocityYSpeedMetersPerSecond The desired speed on the X axis in meters per second.
   * @param yaw The current yaw of the robot, if in tolerance, pass in the setpoint instead.
   * @param yawSetpoint The setpoint to use for the yaw PID controller.
   * @param hasTarget Whether or not there is a target.
   */
  public void driveTargetLock(
      double velocityXSpeedMetersPerSecond,
      double velocityYSpeedMetersPerSecond,
      double yaw,
      double yawSetpoint,
      boolean hasTarget) {
    double vy = getFieldRelativeChassisSpeeds().vyMetersPerSecond; // Horizontal velocity
    double kp = Constants.SWERVE.TARGET_LOCK_ROTATION_KP;
    kp *= Math.max(1, vy * 1);
    Constants.SWERVE.TARGET_LOCK_ROTATION_PID_CONTROLLER.setP(kp);

    double rotation =
        Constants.SWERVE.TARGET_LOCK_ROTATION_PID_CONTROLLER.calculate(yaw, yawSetpoint);
    // if we have a target, add feedforward to the controls, if we dont, let the driver rotate the
    // robot manually.
    double rotationOutput =
        !hasTarget
            ? Robot.driverControls.getRotation() * SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND
            : rotation + Math.copySign(Constants.SWERVE.TARGET_LOCK_FEED_FORWARD, rotation);
    driveFieldRelative(
        velocityXSpeedMetersPerSecond, velocityYSpeedMetersPerSecond, rotationOutput);
  }

  /**
   * The method to use for robot relative driving.
   *
   * @param velocityXSpeedMetersPerSecond The desired speed on the X axis in meters per second.
   * @param velocityYSpeedMetersPerSecond The desired speed on the X axis in meters per second.
   * @param rotationRateRadiansPerSecond The desired rotation rate in radians per second.
   */
  public void driveRobotRelative(
      double velocityXMetersPerSecond,
      double velocityYMetersPerSecond,
      double rotationRateRadiansPerSecond) {
    applyRequest(
        () ->
            robotRelative
                .withVelocityX(velocityXMetersPerSecond)
                .withVelocityY(velocityYMetersPerSecond)
                .withRotationalRate(rotationRateRadiansPerSecond));
  }

  /**
   * The method to use for field relative driving.
   *
   * @param velocityXSpeedMetersPerSecond The desired speed on the X axis in meters per second.
   * @param velocityYSpeedMetersPerSecond The desired speed on the X axis in meters per second.
   * @param rotationRateRadiansPerSecond The desired rotation rate in radians per second.
   */
  public void driveFieldRelative(
      double velocityXMetersPerSecond,
      double velocityYMetersPerSecond,
      double rotationRateRadiansPerSecond) {
    applyRequest(
        () ->
            fieldRelative
                .withVelocityX(velocityXMetersPerSecond)
                .withVelocityY(velocityYMetersPerSecond)
                .withRotationalRate(rotationRateRadiansPerSecond));
  }

  /**
   * @return A list of module positions in the order Front Left, Front Right, Back Left, Back Right
   */
  public Translation2d[] getModulePositions() {
    return super.getModuleLocations();
  }

  /**
   * @return A list of module states in the order Front Left, Front Right, Back Left, Back Right
   */
  public SwerveModuleState[] getModuleStates() {
    return super.getState().ModuleStates;
  }

  /**
   * @return A list of module targets in the order Front Left, Front Right, Back Left, Back Right
   */
  public SwerveModuleState[] getModuleTargets() {
    return super.getState().ModuleTargets;
  }

  public Pose2d getPose() {
    return super.getState().Pose;
  }

  public void zeroGyro(boolean flip) {
    StatusCode code = super.getPigeon2().setYaw(flip ? 180 : 0);
    System.out.println("[GYRO] Zeroed to " + (flip ? 180 : 0) + ": " + code.toString());
  }

  public void resetPose() {
    setPose(new Pose2d(0, 0, new Rotation2d()));
  }

  public void stopMotorsIntoX() {
    applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
  }

  /** Stops the motors in a way that should make them not jingle. */
  public void stopMotors() {
    driveFieldRelative(0, 0, 0);
    for (SwerveModule module : super.getModules()) {
      module.getDriveMotor().stopMotor(); // anti-jingle
      module.getSteerMotor().stopMotor(); // remove to bring back the jingle (dont do it)
    }
  }

  public void setPose(Pose2d poseToSet) {
    super.resetPose(poseToSet);
  }

  /**
   * DO NOT USE THIS OUTSIDE OF AUTO DRIVING! This is intended for Choreo only!
   */
  public void autonDriveWithFeedForward(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = getKinematics().toSwerveModuleStates(speeds);
    for (SwerveModuleState state : moduleStates) {
      state.speedMetersPerSecond += Constants.SWERVE.STATIC_FEEDFORWARD_METERS_PER_SECOND;
    }
    setControl(robotSpeedRequest.withSpeeds(getKinematics().toChassisSpeeds(moduleStates)));
  }

  /**
   * Consumes ChassisSpeeds to lockonto the speaker, for overriding Choreo heading.
   *
   * DO NOT USE OUTSIDE OF AUTO STUFF
   */
  public void autonDriveWithTargetLock(ChassisSpeeds speeds) {
        double rotation = getAutonSpeakerLockRadiansPerSecond();
        if (!Double.isNaN(rotation)) {
          speeds.omegaRadiansPerSecond = rotation;
        }

        SwerveModuleState[] moduleStates = getKinematics().toSwerveModuleStates(speeds);
        for (SwerveModuleState state : moduleStates) {
          state.speedMetersPerSecond += Constants.SWERVE.STATIC_FEEDFORWARD_METERS_PER_SECOND;
        }
        setControl(robotSpeedRequest.withSpeeds(getKinematics().toChassisSpeeds(moduleStates)));
      }
  /**
   * Gets the radians per second to turn in for target locking in auto.
   *
   * @return The radians per second to turn for target lcoking on the speaker.
   */
  public double getAutonSpeakerLockRadiansPerSecond() {
    double targetPitch = Robot.shooterCam.getSpeakerTargetPitch();
    double targetYaw = Robot.shooterCam.getSpeakerTargetYaw();

    if (Double.isNaN(targetPitch) || Double.isNaN(targetYaw)) return Double.NaN;

    double yawOffset = Robot.swerve.updateVisionTargeting(targetPitch, 0);

    double vy = getFieldRelativeChassisSpeeds().vyMetersPerSecond; // Horizontal velocity
    double kp = Constants.SWERVE.TARGET_LOCK_ROTATION_KP;
    kp *= Math.max(1, vy * 1);
    Constants.SWERVE.TARGET_LOCK_ROTATION_PID_CONTROLLER.setP(kp);

    double rotation =
        Constants.SWERVE.TARGET_LOCK_ROTATION_PID_CONTROLLER.calculate(targetYaw, yawOffset);

    double radiansPerSecond =
        rotation + Math.copySign(Constants.SWERVE.TARGET_LOCK_FEED_FORWARD, rotation);
    return radiansPerSecond;
  }

  /**
   * Gets the yaw to turn the robot to for target locking so we shoot into the speaker.
   *
   * @param pitch The current pitch of the middle speaker april tag.
   * @param defualtOffset The default offset to return if we dont have a valid curve index.
   * @return The yaw to turn the robot to for target locking.
   */
  public double updateVisionTargeting(double pitch, double defaultOffset) {
    int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, pitch);
    if (curveIndex == -1) {
      return defaultOffset;
    }

    double[] high = Robot.shooterCurve[curveIndex];
    double[] low = Robot.shooterCurve[curveIndex + 1];

    double highPitch = high[0];
    double lowPitch = low[0];
    double highYawSetopint = high[3];
    double lowYawSetpoint = low[3];

    return RobotMath.linearlyInterpolate(
        highYawSetopint, lowYawSetpoint, highPitch, lowPitch, pitch);
  }

  /**
   * Returns the current field relative speeds but in a ChassisSpeeds object.
   *
   * @return the current field relative speeds in a ChassisSpeeds object.
   */
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    ChassisSpeeds chassisSpeeds = getKinematics().toChassisSpeeds(getModuleStates());
    return chassisSpeeds;
  }

  /**
   * Returns the current robot relative speeds but in a ChassisSpeeds object.
   *
   * @return the current robot relative speeds in a ChassisSpeeds object.
   */
  public ChassisSpeeds getRobotRelativChassisSpeeds() {
    var chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            getFieldRelativeChassisSpeeds(), getRotation3d().toRotation2d());
    return chassisSpeeds;
  }

  /**
   * Gets the angle of every module in degrees
   *
   * @return A list of every modules current angle in degrees.
   */
  public double[] getWheelRadiusCharacterizationPosition() {
    double[] positions = new double[4];

    for (int i = 0; i < positions.length; i++) {
      positions[i] = getModulePositions()[i].getAngle().getDegrees();
    }

    return positions;
  }
}
