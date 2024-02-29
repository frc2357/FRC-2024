package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.PhotonVisionCamera;
import frc.robot.util.Utility;

public class DriveToApriltag extends Command {
  private double m_tyOffset;
  private double m_rotationGoal;
  private int m_pipelineIndex;
  private Debouncer m_canSeePieceDebouncer;

  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;

  private PhotonVisionCamera m_camera;

  private boolean m_invertSpeeds;

  public DriveToApriltag(
      double tyOffset,
      double rotationGoal,
      int pipelineIndex,
      PhotonVisionCamera camera,
      boolean invertSpeeds) {
    m_tyOffset = tyOffset;
    m_rotationGoal = rotationGoal;
    m_pipelineIndex = pipelineIndex;
    m_xController = Constants.SWERVE.APRILTAG_X_TRANSLATION_PID_CONTROLLER;
    m_yController = Constants.SWERVE.APRILTAG_Y_TRANSLATION_PID_CONTROLLER;
    m_rotationController = Constants.SWERVE.APRILTAG_ROTATION_PID_CONTROLLER;
    m_camera = camera;
    m_invertSpeeds = invertSpeeds;
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    // trys to save camera time if the camera alrady has the desired pipeline selected.
    if (m_camera.getPipeline() != m_pipelineIndex) {
      m_camera.setPipeline(m_pipelineIndex);
    }
    // reset pids
    m_yController.setSetpoint(m_tyOffset + Constants.SWERVE.APRILTAG_TY_MAGIC_OFFSET);
    m_yController.reset();
    m_xController.setSetpoint(Constants.SWERVE.AMP_TX_SETPOINT);
    m_xController.reset();

    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.setSetpoint(m_rotationGoal);
    m_rotationController.reset();

    m_canSeePieceDebouncer =
        new Debouncer(Constants.SWERVE.AUTO_TRANSLATE_DEBOUNCE_SECONDS, DebounceType.kFalling);
  }

  @Override
  public void execute() {
    if (!m_canSeePieceDebouncer.calculate(m_camera.validTargetExists())) {
      System.out.println("No Target Detected");
      Robot.swerve.driveRobotRelative(m_tyOffset, m_rotationGoal, m_pipelineIndex);
      return;
    }

    double tx = m_camera.getTX();
    double ty = m_camera.getTY();
    if (tx == Double.NaN) {
      tx = 0;
    }
    if (ty == Double.NaN) {
      ty = 0;
    }
    double rotationError = Robot.swerve.getPose().getRotation().getRadians();

    // Increase tx tolerance when close to target since tx is more sensitive at
    // shorter distances
    double txTolerance = Constants.SWERVE.APRILTAG_X_TOLERANCE;
    if (Utility.isWithinTolerance(ty, m_tyOffset, 4)) {
      txTolerance = Math.copySign(txTolerance * 2, txTolerance);
    }

    if (Utility.isWithinTolerance(rotationError, 0, Constants.SWERVE.APRILTAG_ROTATION_TOLERANCE)) {
      rotationError = 0;
    }

    // Reduce tx based on how far off our rotation is so the x controller doesn't
    // over compensate
    tx -= Rotation2d.fromRadians(rotationError).getDegrees();
    if (Utility.isWithinTolerance(tx, 0, txTolerance)) {
      tx = 0;
    }
    if (Utility.isWithinTolerance(ty, m_tyOffset, Constants.SWERVE.APRILTAG_Y_TOLERANCE)) {
      ty = m_tyOffset;
    }
    double xMetersPerSecond =
        m_yController.calculate(ty + Constants.SWERVE.APRILTAG_TY_MAGIC_OFFSET)
            * (m_invertSpeeds ? -1 : 1);
    double yMetersPerSecond = m_xController.calculate(tx) * (m_invertSpeeds ? -1 : 1);
    double rotationRadiansPerSecond = m_rotationController.calculate(rotationError);
    Robot.swerve.driveRobotRelative(m_tyOffset, m_rotationGoal, m_pipelineIndex);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.driveRobotRelative(m_tyOffset, m_rotationGoal, m_pipelineIndex);
  }
}
