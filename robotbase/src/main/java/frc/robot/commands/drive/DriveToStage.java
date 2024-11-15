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
  private PIDController m_yawController;
  private PIDController m_pitchController;
  private PIDController m_rotationController;

  public DriveToStage() {
    m_yawController = SWERVE.VISION_X_TRANSLATION_PID_CONTROLLER;
    m_pitchController = SWERVE.VISION_Y_TRANSLATION_PID_CONTROLLER;
    m_rotationController = SWERVE.PIGEON_ROTATION_PID_CONTROLLER;
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    Robot.shooterCam.setAprilTagPipelineActive();

    // reset pids
    m_pitchController.setTolerance(SWERVE.VISION_PITCH_TOLERANCE);
    m_pitchController.setSetpoint(SWERVE.STAGE_PITCH_SETPOINT);
    m_pitchController.reset();
    m_yawController.setTolerance(SWERVE.VISION_YAW_TOLERANCE);
    m_yawController.setSetpoint(SWERVE.STAGE_YAW_SETPOINT);
    m_yawController.reset();

    m_rotationController.setTolerance(SWERVE.VISION_ROTATION_TOLERANCE_RADIANS);
    m_rotationController.setSetpoint(DriveUtility.getStageRotationGoal());
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.reset();
  }

  @Override
  public void execute() {
    // Rotation
    double rotation =
        DriveUtility.calculateRotationError(
            Robot.swerve.getPose().getRotation().getRadians(), m_rotationController.getSetpoint());
    double rotationRadiansPerSecond = m_rotationController.calculate(rotation);
    double rotationFeedforward =
        Math.copySign(SWERVE.PIGEON_ROTATION_FEEDFORWARD, rotationRadiansPerSecond);

    rotationRadiansPerSecond += rotationFeedforward;
    if (rotation == m_rotationController.getSetpoint()) {
      rotationRadiansPerSecond = 0;
    }

    // Translation
    double yaw = Robot.shooterCam.getStageTargetYaw();

    // If rotation goal hasn't been set, try again
    if (Double.isNaN(m_rotationController.getSetpoint())) {
      m_rotationController.setSetpoint(DriveUtility.getStageRotationGoal());
    }
    if (Double.isNaN(yaw)) {
      System.out.println("[DriveToStage] No STAGE Detected");
      Robot.swerve.driveRobotRelative(0.0, 0.0, rotationRadiansPerSecond);
      return;
    }

    double pitch = Robot.shooterCam.getStageTargetPitch();

    yaw =
        DriveUtility.adjustYawForApriltag(
            yaw,
            pitch,
            rotation - m_rotationController.getSetpoint(),
            SWERVE.STAGE_PITCH_SETPOINT,
            SWERVE.VISION_CLOSE_PITCH,
            SWERVE.VISION_YAW_TOLERANCE);
    pitch =
        DriveUtility.adjustPitchForApriltag(
            pitch, SWERVE.STAGE_PITCH_SETPOINT, SWERVE.VISION_PITCH_TOLERANCE);

    double xMetersPerSecond = m_yawController.calculate(yaw);
    double yMetersPerSecond = m_pitchController.calculate(pitch);
    // Robot.swerve.driveRobotRelative(0, 0, rotationRadiansPerSecond);
    Robot.swerve.driveRobotRelative(-yMetersPerSecond, -xMetersPerSecond, rotationRadiansPerSecond);
  }

  @Override
  public boolean isFinished() {
    return (m_pitchController.atSetpoint()
        && m_yawController.atSetpoint()
        && m_rotationController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }
}
