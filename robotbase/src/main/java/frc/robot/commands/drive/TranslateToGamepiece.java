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
  private long m_startTime;
  private double m_startingSpeed;
  private boolean m_targetBad = false;

  public TranslateToGamepiece(double startingSpeed) {
    m_startingSpeed = startingSpeed;
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
    m_startTime = System.currentTimeMillis();

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
    // Linear deceleration y direction
    double secondsElapsed = (double)(System.currentTimeMillis() - m_startTime) / 1000.0;

    double percentSpeed = 1 - ((double)secondsElapsed / (double)SWERVE.TRANSLATE_TO_GAMEPIECE_Y_DURATION_SECONDS);
    double yMetersPerSecond = percentSpeed * m_startingSpeed;

    double yaw = Robot.intakeCam.getNoteTargetYaw();
    double pitch = Robot.intakeCam.getNoteTargetPitch();

    if (Double.isNaN(yaw) || m_targetBad) {
      System.out.println("[TranslateToGamepiece] No gamepiece detected");
      // Continue driving forward and rotating even if we don't see a gamepiece
      Robot.swerve.driveRobotRelative(yMetersPerSecond, 0, 0);
    }

    if (Utility.isWithinTolerance(
        yaw, m_yawController.getSetpoint(), SWERVE.TRANSLATE_TO_GAMEPIECE_YAW_TOLERANCE)) {
      yaw = m_yawController.getSetpoint();
    }

    double xMetersPerSecond = m_yawController.calculate(yaw);
    if (pitch < -16 || pitch > 5) {
      m_targetBad = true;
      xMetersPerSecond = 0;
    }

    Robot.swerve.driveRobotRelative(yMetersPerSecond, xMetersPerSecond, 0);
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
