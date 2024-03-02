package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Utility;

public class DriveToApriltag extends Command {
  private double m_tySetpoint;
  private double m_txSetpoint;
  private Supplier<Double> m_rotationGoalSupplier;
  private int[] m_tags;

  private Debouncer m_canSeePieceDebouncer;

  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;

  public DriveToApriltag(double tySetpoint, double txSetpoint, Supplier<Double> rotationGoal, int[] tags) {
    m_tySetpoint = tySetpoint;
    m_txSetpoint = txSetpoint;
    m_rotationGoalSupplier = rotationGoal;
    m_tags = tags;

    m_xController = Constants.SWERVE.APRILTAG_X_TRANSLATION_PID_CONTROLLER;
    m_yController = Constants.SWERVE.APRILTAG_Y_TRANSLATION_PID_CONTROLLER;
    m_rotationController = Constants.SWERVE.APRILTAG_ROTATION_PID_CONTROLLER;
    addRequirements(Robot.swerve, Robot.shooterCam);
  }

  @Override
  public void initialize() {
    // reset pids
    Robot.shooterCam.setAprilTagPipelineActive();
    m_yController.setSetpoint(m_tySetpoint + Constants.SWERVE.APRILTAG_TY_MAGIC_OFFSET);
    m_yController.reset();
    m_xController.setSetpoint(m_txSetpoint);
    m_xController.reset();

    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.setSetpoint(m_rotationGoalSupplier.get());
    m_rotationController.reset();

    m_canSeePieceDebouncer =
        new Debouncer(Constants.SWERVE.AUTO_TRANSLATE_DEBOUNCE_SECONDS, DebounceType.kFalling);
  }

  @Override
  public void execute() {
    ArrayList<PhotonTrackedTarget> targets = Robot.shooterCam.filterAprilTags(m_tags);
    if (targets == null || targets.size() == 0) {
      System.out.println("No Target Detected");
      Robot.swerve.stopMotors();
      return;
    }
    if (Double.isNaN(m_rotationController.getSetpoint())) {
      System.err.println("Could not calculate Rotation Setpoint");
      m_rotationController.setSetpoint(m_rotationGoalSupplier.get());
      return;
    }

    PhotonTrackedTarget target = targets.get(0);

    double tx = target.getYaw();
    double ty = target.getPitch();
    double yaw = Robot.swerve.getPose().getRotation().getDegrees();
    double rotationError = Rotation2d.fromDegrees(m_rotationController.getSetpoint() - yaw).getRadians();

    // Increase tx tolerance when close to target since tx is more sensitive at
    // shorter distances
    double txTolerance = Constants.SWERVE.APRILTAG_X_TOLERANCE;
    if (Utility.isWithinTolerance(ty, m_tySetpoint, 4)) {
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
    if (Utility.isWithinTolerance(ty, m_tySetpoint, Constants.SWERVE.APRILTAG_Y_TOLERANCE)) {
      ty = m_tySetpoint;
    }
    double xMetersPerSecond =
        -m_yController.calculate(ty + Constants.SWERVE.APRILTAG_TY_MAGIC_OFFSET);
    double yMetersPerSecond = -m_xController.calculate(tx);
    double rotationRadiansPerSecond = m_rotationController.calculate(rotationError);
    Robot.swerve.driveRobotRelative(0, 0, rotationRadiansPerSecond);
    // Robot.swerve.driveRobotRelative(yMetersPerSecond, xMetersPerSecond, rotationRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }
}
