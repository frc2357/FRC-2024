package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

public class DriveToAmp extends Command {
  private double m_pitchOffset;

  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;

  public DriveToAmp(double pitchOffset, double rotationGoal) {
    m_pitchOffset = pitchOffset;
    m_xController = SWERVE.APRILTAG_X_TRANSLATION_PID_CONTROLLER;
    m_yController = SWERVE.APRILTAG_Y_TRANSLATION_PID_CONTROLLER;
    m_rotationController = SWERVE.APRILTAG_ROTATION_PID_CONTROLLER;
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    Robot.shooterCam.setAprilTagPipelineActive();

    // reset pids
    m_yController.setSetpoint(m_pitchOffset + SWERVE.AMP_PITCH_SETPOINT);
    m_yController.reset();
    m_xController.setSetpoint(SWERVE.AMP_YAW_SETPOINT);
    m_xController.reset();

    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.setSetpoint(SWERVE.AMP_ROTATION_SETPOINT);
    m_rotationController.reset();
  }

  @Override
  public void execute() {
    double yaw = Robot.shooterCam.getAmpTargetYaw();

    if (Double.isNaN(yaw)) {
      System.out.println("[DrivtToAmp] No AMP Detected");
      return;
    }

    double pitch = Robot.shooterCam.getAmpTargetPitch();
    double rotationError =
        DriveUtility.calculateRotationError(Robot.swerve.getPose().getRotation().getRadians());

    yaw =
        DriveUtility.adjustYawForApriltag(
            yaw,
            pitch,
            rotationError,
            m_pitchOffset,
            SWERVE.APRILTAG_CLOSE_PITCH,
            SWERVE.APRILTAG_YAW_TOLERANCE);
    pitch =
        DriveUtility.adjustPitchForApriltag(pitch, m_pitchOffset, SWERVE.APRILTAG_PITCH_TOLERANCE);

    double xMetersPerSecond = m_yController.calculate(pitch + SWERVE.APRILTAG_PITCH_MAGIC_OFFSET);
    double yMetersPerSecond = m_xController.calculate(yaw);
    double rotationRadiansPerSecond = m_rotationController.calculate(rotationError);
    Robot.swerve.driveRobotRelative(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }
}
