package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

/*
 * New Plan:
 * Rotate and horizontal translation using apriltag then robot relative forward translation to apriltag
 */
public class DriveToStage extends Command {
  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;

  public DriveToStage() {
    m_xController = SWERVE.APRILTAG_X_TRANSLATION_PID_CONTROLLER;
    m_yController = SWERVE.APRILTAG_Y_TRANSLATION_PID_CONTROLLER;
    m_rotationController = SWERVE.APRILTAG_ROTATION_PID_CONTROLLER;
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    Robot.shooterCam.setAprilTagPipelineActive();

    // reset pids
    m_yController.setSetpoint(SWERVE.STAGE_PITCH_SETPOINT);
    m_yController.reset();
    m_xController.setSetpoint(SWERVE.STAGE_YAW_SETPOINT);
    m_xController.reset();

    m_rotationController.setSetpoint(DriveUtility.getStageRotationGoal());
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.reset();
  }

  @Override
  public void execute() {
    // Rotation
    double rotationError = DriveUtility.calculateRotationError(Robot.swerve.getPose().getRotation().getRadians(),
        m_rotationController.getSetpoint());
    double rotationRadiansPerSecond = m_rotationController.calculate(rotationError);
    double rotationFeedforward = Math.copySign(SWERVE.APRILTAG_ROTATION_FEEDFORWARD, rotationRadiansPerSecond);

    rotationRadiansPerSecond += rotationFeedforward;
    if (rotationError == m_rotationController.getSetpoint()) {
      rotationRadiansPerSecond = 0;
    }

    // Translation
    double yaw = Robot.shooterCam.getStageTargetYaw();

    // If rotation goal hasn't been set, try again
    if (Double.isNaN(m_rotationController.getSetpoint())) {
      m_rotationController.setSetpoint(DriveUtility.getStageRotationGoal());
    }
    if (Double.isNaN(yaw)) {
      System.out.println("[DrivtToStage] No STAGE Detected");
      Robot.swerve.driveRobotRelative(0.0, 0.0, rotationRadiansPerSecond);
      return;
    }

    double pitch = Robot.shooterCam.getStageTargetPitch();

    yaw = DriveUtility.adjustYawForApriltag(
        yaw,
        pitch,
        rotationError - m_rotationController.getSetpoint(),
        SWERVE.STAGE_PITCH_SETPOINT,
        SWERVE.APRILTAG_CLOSE_PITCH,
        SWERVE.APRILTAG_YAW_TOLERANCE);
    pitch = DriveUtility.adjustPitchForApriltag(
        pitch, SWERVE.STAGE_PITCH_SETPOINT, SWERVE.APRILTAG_PITCH_TOLERANCE);

    double xMetersPerSecond = m_xController.calculate(yaw);
    double yMetersPerSecond = m_yController.calculate(pitch);
    // Robot.swerve.driveRobotRelative(0, 0, rotationRadiansPerSecond);
    Robot.swerve.driveRobotRelative(-yMetersPerSecond, -xMetersPerSecond, rotationRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }
}
