package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    super.tareEverything();
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    this(driveTrainConstants, 0, modules);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * @return A list of the module positions in the order Front Left, Front Right, Back Left, Back
   *     right
   */
  public SwerveModulePosition[] getModulePositions() {
    return super.m_modulePositions;
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

  public void zeroOdometryAndPose() {
    super.tareEverything();
  }

  public void zeroGyro() {
    super.getPigeon2().reset();
  }

  public void stopMotors() {
    SwerveModule[] modules = super.Modules;
    for (int i = 0; i < modules.length; i++) {
      modules[i].apply(
          new SwerveModuleState(0, modules[i].getCurrentState().angle), DriveRequestType.Velocity);
    }
  }

  /**
   * Sets the pose of the robot to the specified position, this is for field-relative maneuvers.
   *
   * @param poseToSet The pose that is set for the swerve
   */
  public void setPose(Pose2d poseToSet) {
    super.seedFieldRelative(poseToSet);
  }

  public Consumer<ChassisSpeeds> getChassisSpeedsConsumer() {
    SwerveModule[] modules = super.Modules;
    SwerveDriveKinematics kinematics = super.m_kinematics;
    return new Consumer<ChassisSpeeds>() {
      @Override
      public void accept(ChassisSpeeds speeds) {
        for (int i = 0; i < modules.length; i++) {
          SwerveModuleState[] statesToApply = kinematics.toSwerveModuleStates(speeds);
          modules[i].apply(statesToApply[i], DriveRequestType.OpenLoopVoltage);
        }
      }
    };
  }
}
