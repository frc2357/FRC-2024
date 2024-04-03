package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

  private final SwerveRequest.ApplyChassisSpeeds chassisSpeedRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  // Comment out below requests for CUBE_BOT
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
    double rotationOutput =
        !hasTarget
            ? Robot.driverControls.getRotation() * SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND
            : rotation + Math.copySign(Constants.SWERVE.TARGET_LOCK_FEED_FORWARD, rotation);
    driveFieldRelative(
        velocityXSpeedMetersPerSecond, velocityYSpeedMetersPerSecond, rotationOutput);
  }

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
   * @return A list of the module positions in the order Front Left, Front Right, Back Left, Back
   *     Right
   */
  public SwerveModulePosition[] getModulePositions() {
    return super.m_modulePositions;
  }

  public SwerveModuleState[] getModuleStates() {
    return super.getState().ModuleStates;
  }

  public SwerveModuleState[] getModuleTargets() {
    return super.getState().ModuleTargets;
  }

  public Pose2d getPose() {
    return super.getState().Pose;
  }

  private SwerveDriveKinematics getKinematics() {
    return super.m_kinematics;
  }

  public void zeroGyro(boolean flip) {
    // Pigeon2Configuration config = new Pigeon2Configuration();
    // super.getPigeon2().getConfigurator().apply(config);
    StatusCode code = super.getPigeon2().setYaw(flip ? 180 : 0);
    System.out.println("[GYRO] Zeroed to " + (flip ? 180 : 0) + ": " + code.toString());
  }

  public void resetPose() {
    setPose(new Pose2d(0, 0, new Rotation2d()));
  }

  public void stopMotorsIntoX() {
    applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
  }

  public void stopMotors() {
    driveFieldRelative(0, 0, 0);
    for (SwerveModule module : super.Modules) {
      module.getDriveMotor().stopMotor();
      module.getSteerMotor().stopMotor();
    }
  }

  public void setPose(Pose2d poseToSet) {
    super.seedFieldRelative(poseToSet);
    // var statusCode = super.m_pigeon2.setYaw(poseToSet.getRotation().getDegrees());
    // if(!statusCode.isOK()){
    //   System.err.println("[CommandSwerveDrivetrain] SET POSE - PIGEON DID NOT SET YAW CORRECTLY.
    // \n\tSTATUS CODE DESCRIP: " + statusCode.getDescription());
    // }
  }

  public Consumer<ChassisSpeeds> getChassisSpeedsConsumer() {
    return new Consumer<ChassisSpeeds>() {
      @Override
      public void accept(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = getKinematics().toSwerveModuleStates(speeds);
        for (SwerveModuleState state : moduleStates) {
          state.speedMetersPerSecond += Constants.SWERVE.STATIC_FEEDFORWARD_METERS_PER_SECOND;
        }
        setControl(chassisSpeedRequest.withSpeeds(getKinematics().toChassisSpeeds(moduleStates)));
      }
    };
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    ChassisSpeeds chassisSpeeds = getKinematics().toChassisSpeeds(getModuleStates());
    return chassisSpeeds;
  }

  public ChassisSpeeds getRobotRelativChassisSpeeds() {
    var chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            getFieldRelativeChassisSpeeds(), getRotation3d().toRotation2d());
    return chassisSpeeds;
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    double[] positions = new double[4];

    for (int i = 0; i < positions.length; i++) {
      positions[i] = getModulePositions()[i].angle.getDegrees();
    }

    return positions;
  }
}
