package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INTAKE_PHOTON_CAMERA;
import frc.robot.Constants.SWERVE;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Utility;

public class TranslateToGamepiece extends Command {
  private PIDController m_yawController;
  private PIDController m_rotationController;
  private Timer m_timer;
  private long m_startTime;
  private double m_startingSpeed;
  private double m_decelSlope;
  private double m_decelIntercept;
  private boolean m_targetBad = false;
  private double m_startRotation;
  private double m_timeToRunSeconds;

  public TranslateToGamepiece(double startingSpeed){
    this(startingSpeed, SWERVE.TRANSLATE_TO_GAMEPIECE_Y_DURATION_SECONDS);
  }
  public TranslateToGamepiece(double startingSpeed, double timeToRunSeconds) {
    m_startingSpeed = startingSpeed;
    m_yawController = SWERVE.VISION_X_TRANSLATION_PID_CONTROLLER;
    m_rotationController = SWERVE.PIGEON_ROTATION_PID_CONTROLLER;
    m_timer = new Timer();
    m_timeToRunSeconds = timeToRunSeconds;
    addRequirements(Robot.swerve, Robot.intakeCam);
  }

  @Override
  public void initialize() {
    Robot.intakeCam.setPipeline(INTAKE_PHOTON_CAMERA.NEURAL_NETWORK_PIPELINE);

    m_startRotation = Robot.swerve.getYaw();

    // Initialize decel variables
    m_decelSlope =
        (SWERVE.TRANSLATE_TO_GAMEPIECE_MIN_SPEED_METERS_PER_SECOND - m_startingSpeed)
            / (m_timeToRunSeconds
                * SWERVE.TRANSLATE_TO_GAMEPIECE_START_DECEL_THRESHOLD);
    m_decelIntercept =
        m_startingSpeed
            - m_decelSlope
                * (m_timeToRunSeconds
                    - SWERVE.TRANSLATE_TO_GAMEPIECE_Y_DURATION_SECONDS
                        * SWERVE.TRANSLATE_TO_GAMEPIECE_START_DECEL_THRESHOLD);

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
    // deceleration y direction
    double yMetersPerSecond = calculateYSpeed();

    double currentRotation = Robot.swerve.getYaw();
    double rotationError = currentRotation - m_startRotation;
    double rotationOutput =
        m_rotationController.calculate(Rotation2d.fromDegrees(rotationError).getRadians());
    if (Utility.isWithinTolerance(
        currentRotation, m_startRotation, SWERVE.TRANSLATE_TO_GAMEPIECE_YAW_TOLERANCE)) {
      rotationOutput = 0;
    }

    double yaw = Robot.intakeCam.getNoteTargetYaw();
    double pitch = Robot.intakeCam.getNoteTargetPitch();

    if (Double.isNaN(yaw) || m_targetBad) {
      System.out.println("[TranslateToGamepiece] No gamepiece detected");
      // Continue driving forward and rotating even if we don't see a gamepiece
      Robot.swerve.driveRobotRelative(yMetersPerSecond, 0, rotationOutput);
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

    System.out.println("y: " + yMetersPerSecond + ", x: " + xMetersPerSecond);
    Robot.swerve.driveRobotRelative(yMetersPerSecond, xMetersPerSecond, rotationOutput);
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_timeToRunSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }

  private double calculateYSpeed() {
    double secondsElapsed = (System.currentTimeMillis() - m_startTime) / 1000.0;
    double percentDone = secondsElapsed / m_timeToRunSeconds;
    if (1 - percentDone >= SWERVE.TRANSLATE_TO_GAMEPIECE_START_DECEL_THRESHOLD) {
      return m_startingSpeed;
    }

    double decelledSpeed = m_decelSlope * percentDone + m_decelIntercept;
    return decelledSpeed;
  }
}
