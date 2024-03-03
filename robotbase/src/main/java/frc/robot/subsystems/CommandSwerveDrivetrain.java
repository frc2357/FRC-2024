package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.util.ModifiedSignalLogger;
import frc.robot.util.SwerveVoltageRequest;
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

  private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);

  // Spectrum's config
  private SysIdRoutine.Config sysidConfig = new SysIdRoutine.Config(
      Volts.of(1).per(Seconds.of(0.5)), // Default ramp rate is acceptable
      Volts.of(10), // Reduce dynamic voltage to 4 to prevent motor brownout
      Seconds.of(5), (state) -> SignalLogger.writeString("state", state.toString()) // Default timeout is acceptable
  );


  private SysIdRoutine m_driveSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) ->
                  setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
              null,
              this));

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

  public void driveTargetLock(
      double velocityXSpeedMetersPerSecond,
      double velocityYSpeedMetersPerSecond,
      double tx,
      boolean hasTarget) {
    double vy = getChassisSpeeds().vyMetersPerSecond; // Horizontal velocity
    double kp = Constants.SWERVE.TARGET_LOCK_ROTATION_KP;
    kp *= Math.max(1, vy * 1);
    Constants.SWERVE.TARGET_LOCK_ROTATION_PID_CONTROLLER.setP(kp);

    double rotation =
        Constants.SWERVE.TARGET_LOCK_ROTATION_PID_CONTROLLER.calculate(
            tx, SWERVE.TARGET_LOCK_YAW_SETPOINT);
    double rotationOutput =
        !hasTarget
            ? Robot.driverControls.getRotation()
            : rotation + Math.copySign(Constants.SWERVE.TARGET_LOCK_FEED_FORWARD, rotation);
    // System.out.println("Rotation: " + rotation);
    // System.out.println("Rotation output: " + rotationOutput);
    // System.out.println("Yaw: " + tx);
    applyRequest(
        () ->
            fieldRelative
                .withVelocityX(velocityXSpeedMetersPerSecond)
                .withVelocityY(velocityYSpeedMetersPerSecond)
                .withRotationalRate(rotationOutput));
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
    driveFieldRelative(0, 0, 0);
    for (SwerveModule module : super.Modules) {
      module.getDriveMotor().stopMotor();
      module.getSteerMotor().stopMotor();
    }
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

  // SysID Commands
  public Command runDriveQuasiTest(SysIdRoutine.Direction direction) {
    return m_driveSysIdRoutine.quasistatic(direction);
  }

  public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
    return m_driveSysIdRoutine.dynamic(direction);
  }
}
