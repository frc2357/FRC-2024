package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INTAKE_PHOTON_CAMERA;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.util.Utility;

public class TranslateToGamepiece extends Command {
  private PIDController m_yawController;
  private PIDController m_rotationController;
  private Timer m_timer;

  public TranslateToGamepiece() {
    m_yawController = SWERVE.VISION_X_TRANSLATION_PID_CONTROLLER;
    m_rotationController = SWERVE.PIGEON_ROTATION_PID_CONTROLLER;
    m_timer = new Timer();
    addRequirements(Robot.swerve, Robot.intakeCam);
  }

  @Override
  public void initialize() {
    Robot.intakeCam.setPipeline(INTAKE_PHOTON_CAMERA.NEURAL_NETWORK_PIPELINE);

    m_timer.reset();
    m_timer.start();

    m_yawController.setTolerance(SWERVE.VISION_YAW_TOLERANCE);
    m_yawController.setSetpoint(SWERVE.TRANSLATE_TO_GAMEPIECE_YAW_SETPOINT);
    m_yawController.reset();

    m_rotationController.setTolerance(SWERVE.VISION_ROTATION_TOLERANCE_RADIANS);
    m_rotationController.setSetpoint(SWERVE.TRANSLATE_TO_GAMEPIECE_ROTATION_SETPOINT);
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.reset();
  }

  @Override
  public void execute() {
    double rotationRadiansPerSecond = 0;
    double rotationError =
        DriveUtility.calculateRotationError(
            Robot.swerve.getYaw(), m_rotationController.getSetpoint());
    if (rotationError != m_rotationController.getSetpoint()) {
      rotationRadiansPerSecond = m_rotationController.calculate(rotationError);
      double rotationFeedforward =
          Math.copySign(SWERVE.PIGEON_ROTATION_FEEDFORWARD, rotationRadiansPerSecond);
      rotationRadiansPerSecond += rotationFeedforward;
    }

    double yaw = Robot.intakeCam.getNoteTargetYaw();

    if (Double.isNaN(yaw)) {
      System.out.println("[TranslateToGamepiece] No gamepiece detected");
      // Continue driving forward and rotating even if we don't see a gamepiece
      Robot.swerve.driveFieldRelative(
          SWERVE.TRANSLATE_TO_GAMEPIECE_Y_SPEED_MPS, 0, rotationRadiansPerSecond);
    }

    yaw -= Rotation2d.fromRadians(rotationError).getDegrees();
    if (Utility.isWithinTolerance(
        yaw, m_yawController.getSetpoint(), SWERVE.TRANSLATE_TO_GAMEPIECE_YAW_TOLERANCE)) {
      yaw = m_yawController.getSetpoint();
    }

    double xMetersPerSecond = m_yawController.calculate(yaw);

    Robot.swerve.driveFieldRelative(
        SWERVE.TRANSLATE_TO_GAMEPIECE_Y_SPEED_MPS, xMetersPerSecond, rotationRadiansPerSecond);
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(SWERVE.TRANSLATE_TO_GAMEPIECE_Y_DURATION_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }
}
