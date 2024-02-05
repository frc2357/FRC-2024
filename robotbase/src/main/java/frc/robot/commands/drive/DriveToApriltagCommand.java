package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Utility;

public class DriveToApriltagCommand extends Command {
  private double m_tyOffset;
  private double m_rotationGoal;
  private int m_pipelineIndex;
  private Debouncer m_canSeePieceDebouncer;

  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;

  public DriveToApriltagCommand(double tyOffset, double rotationGoal, int pipelineIndex) {
    m_tyOffset = tyOffset;
    m_rotationGoal = rotationGoal;
    m_pipelineIndex = pipelineIndex;

    m_xController = Constants.SWERVE.APRILTAG_X_TRANSLATION_PID_CONTROLLER;
    m_yController = Constants.SWERVE.APRILTAG_Y_TRANSLATION_PID_CONTROLLER;
    m_rotationController = Constants.SWERVE.APRILTAG_ROTATION_PID_CONTROLLER;

    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    Robot.shooterLimelight.setPipeline(m_pipelineIndex);

    // reset pids
    m_yController.setSetpoint(m_tyOffset + Constants.SWERVE.APRILTAG_TY_MAGIC_OFFSET);
    m_yController.reset();

    m_xController.setSetpoint(0);
    m_xController.reset();

    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.setSetpoint(m_rotationGoal);
    m_rotationController.reset();

    m_canSeePieceDebouncer =
        new Debouncer(Constants.SWERVE.AUTO_TRANSLATE_DEBOUNCE_SECONDS, DebounceType.kFalling);
  }

  @Override
  public void execute() {
    if (!m_canSeePieceDebouncer.calculate(Robot.shooterLimelight.validTargetExists())) {
      System.out.println("No Target Detected");
      Robot.swerve.drive(0, 0, 0);
      return;
    }

    double tx = Robot.shooterLimelight.getTX();
    double ty = Robot.shooterLimelight.getTY();
    double rotationError = Robot.swerve.getPose().getRotation().getRadians();

    // Increase tx tolerance when close to target since tx is more sensitive at
    // shorter distances
    double txTolerance = Constants.SWERVE.APRILTAG_X_TOLERANCE;
    if (Utility.isWithinTolerance(ty, m_tyOffset, 4)) {
      txTolerance *= 2;
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

    Robot.swerve.drive(
        -m_yController.calculate(ty + Constants.SWERVE.APRILTAG_TY_MAGIC_OFFSET),
        m_xController.calculate(tx),
        m_rotationController.calculate(rotationError));
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.drive(0, 0, 0);
  }
}
