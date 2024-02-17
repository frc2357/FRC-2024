package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
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
import frc.robot.Robot;
import frc.robot.util.Utility;
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
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric robotRelative =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Uncomment below for CUBE_BOT
  // private final SwerveRequest.FieldCentric fieldRelative =
  //     new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  // private final SwerveRequest.RobotCentric robotRelative =
  //     new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

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

  public void setYaw(double yaw) {
    getPigeon2().setYaw(yaw);
  }

  public void drive(
      double velocityXMetersPerSecond,
      double velocityYMetersPerSecond,
      double rotationRateRadiansPerSecond) {

    switch (Robot.state.getDriveControlState()) {
      case ROBOT_RELATIVE:
        applyRequest(
            () ->
                robotRelative
                    .withVelocityX(velocityXMetersPerSecond)
                    .withVelocityY(velocityYMetersPerSecond)
                    .withRotationalRate(rotationRateRadiansPerSecond));
        break;
      case FIELD_RELATIVE:
        applyRequest(
            () ->
                fieldRelative
                    .withVelocityX(velocityXMetersPerSecond)
                    .withVelocityY(velocityYMetersPerSecond)
                    .withRotationalRate(rotationRateRadiansPerSecond));
        break;
      case TARGET_LOCK:
        applyRequest(
            () ->
                fieldRelative
                    .withVelocityX(velocityXMetersPerSecond)
                    .withVelocityY(velocityYMetersPerSecond)
                    .withRotationalRate(getTargetLockRotation()));
        break;
    }
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
    return super.m_odometry.getEstimatedPosition();
  }

  public Supplier<Pose2d> getPoseSupplier() {
    return new Supplier<Pose2d>() {

      @Override
      public Pose2d get() {
        return getPose();
      }
    };
  }

  private SwerveDriveKinematics getKinematics() {
    return super.m_kinematics;
  }

  public void zeroAll() {
    zeroGyro();
    resetPose();
  }

  public void zeroGyro() {
    super.getPigeon2().setYaw(0);
  }

  public void resetPose() {
    setPose(new Pose2d(0, 0, new Rotation2d()));
  }

  public void stopMotorsIntoX() {
    applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
  }

  public void stopMotors() {
    drive(0, 0, 0);
  }

  public void setPose(Pose2d poseToSet) {
    super.seedFieldRelative(poseToSet);
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

  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds chassisSpeeds = getKinematics().toChassisSpeeds(getModuleStates());
    return chassisSpeeds;
  }

  public double getTargetLockRotation() {
    double tx = Robot.shooterCam.getTX();
    if (!Robot.shooterCam.validTargetExists()
        || Utility.isWithinTolerance(tx, 0, Constants.SWERVE.TARGET_LOCK_TOLERANCE)) {
      return 0;
    }

    // Increase kP based on horizontal velocity to reduce lag
    double vy = getChassisSpeeds().vyMetersPerSecond; // Horizontal velocity
    double kp = Constants.SWERVE.ROTATION_KP;
    kp *= Math.max(1, vy * 1);
    Constants.SWERVE.ROTATION_PID_CONTROLLER.setP(kp);

    double rotation = -Constants.SWERVE.ROTATION_PID_CONTROLLER.calculate(0, tx);
    double output = rotation + Math.copySign(Constants.SWERVE.TARGET_LOCK_FEED_FORWARD, rotation);
    return output;
  }
}
