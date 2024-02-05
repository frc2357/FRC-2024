package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SHOOTER_LIMELIGHT;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.state.RobotState.DriveControlState;

public class DriveToGamepeiceCommand extends Command {
  private Debouncer m_canSeePieceDebouncer;
  private Pose2d m_initialPose;

  public DriveToGamepeiceCommand() {
    addRequirements(Robot.swerve, Robot.shooterLimelight);
  }

  @Override
  public void initialize() {
    Robot.shooterLimelight.setPipeline(SHOOTER_LIMELIGHT.GAMEPIECE_INDEX);
    Robot.state.setDriveControlState(DriveControlState.ROBOT_RELATIVE);
    SWERVE.ROTATION_PID_CONTROLLER.reset();
    SWERVE.ROTATION_PID_CONTROLLER.setTolerance(SWERVE.PIECE_TRACKING_ROTATION_TOLERANCE);

    m_initialPose = Robot.swerve.getPose();
    m_canSeePieceDebouncer =
        new Debouncer(SWERVE.AUTO_TRANSLATE_DEBOUNCE_SECONDS, DebounceType.kFalling);
  }

  @Override
  public void execute() {
    if (!m_canSeePieceDebouncer.calculate(Robot.shooterLimelight.validTargetExists())) {
      System.out.println("No Gamepiece Detected");
      Robot.swerve.drive(0, 0, 0);
      return;
    }

    double rotationError = Robot.shooterLimelight.getTX();
    double rotationSpeed = SWERVE.ROTATION_PID_CONTROLLER.calculate(rotationError, 0);
    double translationSpeed =
        distanceTraveled() > SWERVE.PIECE_TRACKING_SLOW_DOWN_METERS
            ? SWERVE.PIECE_TRACKING_X_METERS_PER_SECOND / 2.0
            : SWERVE.PIECE_TRACKING_X_METERS_PER_SECOND;

    Robot.swerve.drive(translationSpeed, 0, rotationSpeed);
  }

  @Override
  public boolean isFinished() {
    return isDistanceTraveledTooFar(); // OR GAMEPEICE ACQUIRED
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.drive(0, 0, 0);
    SWERVE.ROTATION_PID_CONTROLLER.setTolerance(0);
    Robot.state.setDriveControlState(DriveControlState.FIELD_RELATIVE);
    System.out.println(interrupted);
  }

  private double distanceTraveled() {
    return Math.abs(
        Robot.swerve.getPose().getTranslation().getDistance(m_initialPose.getTranslation()));
  }

  private boolean isDistanceTraveledTooFar() {
    return Math.abs(distanceTraveled()) > SWERVE.PIECE_TRACKING_MAX_DISTANCE_METERS;
  }
}
