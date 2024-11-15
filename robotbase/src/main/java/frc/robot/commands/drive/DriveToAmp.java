package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

public class DriveToAmp extends Command {
  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;

  public DriveToAmp() {
    m_xController = SWERVE.VISION_X_TRANSLATION_PID_CONTROLLER;
    m_yController = SWERVE.VISION_Y_TRANSLATION_PID_CONTROLLER;
    m_rotationController = SWERVE.PIGEON_ROTATION_PID_CONTROLLER;
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    Robot.shooterCam.setAprilTagPipelineActive();

    // reset pids
    m_yController.setSetpoint(SWERVE.AMP_PITCH_SETPOINT);
    m_yController.reset();
    m_xController.setSetpoint(SWERVE.AMP_YAW_SETPOINT);
    m_xController.reset();

    m_rotationController.setSetpoint(DriveUtility.getAmpRotationGoal());
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.reset();
  }

  @Override
  public void execute() {
    // Rotation
    double rotationError =
        DriveUtility.calculateRotationError(
            Robot.swerve.getPose().getRotation().getRadians(), m_rotationController.getSetpoint());
    double rotationRadiansPerSecond = m_rotationController.calculate(rotationError);
    double rotationFeedforward =
        Math.copySign(SWERVE.PIGEON_ROTATION_FEEDFORWARD, rotationRadiansPerSecond);

    rotationRadiansPerSecond += rotationFeedforward;
    if (rotationError == m_rotationController.getSetpoint()) {
      rotationRadiansPerSecond = 0;
    }

    // Translation
    double yaw = Robot.shooterCam.getAmpTargetYaw();

    // If rotation goal hasn't been set, try again
    if (Double.isNaN(m_rotationController.getSetpoint())) {
      m_rotationController.setSetpoint(DriveUtility.getAmpRotationGoal());
    }
    if (Double.isNaN(yaw)) {
      System.out.println("[DrivtToAmp] No AMP Detected");
      Robot.swerve.driveRobotRelative(0.0, 0.0, rotationRadiansPerSecond);
      return;
    }

    double pitch = Robot.shooterCam.getAmpTargetPitch();

    yaw =
        DriveUtility.adjustYawForApriltag(
            yaw,
            pitch,
            rotationError - m_rotationController.getSetpoint(),
            SWERVE.AMP_PITCH_SETPOINT,
            SWERVE.VISION_CLOSE_PITCH,
            SWERVE.VISION_YAW_TOLERANCE);
    pitch =
        DriveUtility.adjustPitchForApriltag(
            pitch, SWERVE.AMP_PITCH_SETPOINT, SWERVE.VISION_PITCH_TOLERANCE);

    double xMetersPerSecond = m_xController.calculate(yaw);
    double yMetersPerSecond = m_yController.calculate(pitch);
    Robot.swerve.driveRobotRelative(-yMetersPerSecond, -xMetersPerSecond, rotationRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors(); 
  }
}
