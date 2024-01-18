package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SWERVE;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    zeroAll();
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    this(driveTrainConstants, 0, modules);
  }

  public void applyRequest(Supplier<SwerveRequest> requestSupplier) {
    this.setControl(requestSupplier.get());
  }

  public Command applyRequestCommand(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void setYaw(double yaw) {
    getPigeon2().setYaw(yaw);
  }
  
  public void drive(double velocityX, double velocityY, double rotationRate) {
    SwerveRequest driveRequest =
        new SwerveRequest.FieldCentric()
            .withDeadband(SWERVE.MAX_SPEED_METERS_PER_SECOND * SWERVE.SWERVE_TRANSLATIONAL_DEADBAND)
            .withRotationalDeadband(
                SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND * SWERVE.SWERVE_ROTATIONAL_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withRotationalRate(rotationRate);
    applyRequest(() -> driveRequest);
  }

  /**
   * @return A list of the module positions in the order Front Left, Front Right, Back Left, Back
   *     right
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

  public void zeroAll() {
    super.tareEverything();
  }

  public void zeroGyro() {
    super.getPigeon2().reset();
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
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
      }
    };
  }
}
