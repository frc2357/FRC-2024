package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE;
import frc.robot.state.RobotState.DriveControlState;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.Utility;

public class DriveToGamepeiceCommand extends Command {
  private static Limelight limelightSub = Robot.shooterLimelight;
  private static CommandSwerveDrivetrain driveSub = Robot.drive;
  private int hangTimer = 0;

  public DriveToGamepeiceCommand() {
    addRequirements(new Subsystem[] { Robot.drive, Robot.shooterLimelight });
  }

  @Override
  public void initialize() {
    limelightSub.setPipeline(1);
    Robot.state.setDriveControlState(DriveControlState.ROBOT_RELATIVE);
    SWERVE.ROTATION_PID_CONTROLLER.reset();
  }

  @Override
  public void execute() {
    double rotationError = limelightSub.getTX();
    double translationError = limelightSub.getTY();
    double rotationSpeed = SWERVE.ROTATION_PID_CONTROLLER.calculate(rotationError, 0);
    // double translationSpeed = ;
    Robot.drive.drive(0, 0, rotationSpeed);
  }

  @Override
  public boolean isFinished() {
    if (!limelightSub.validTargetExists()) {
      hangTimer++;
    } else {
      hangTimer = 0;
    }

    return (Utility.isWithinTolerance(
        limelightSub.getTX(),
        Constants.SWERVE.GAMEPIECE_TRACKING_ROTATION_OFFSET,
        Constants.SWERVE.GAMEPIECE_TRACKING_ROTATION_TOLERANCE)
        && Utility.isWithinTolerance(
            limelightSub.getTY(),
            Constants.SWERVE.GAMEPIECE_TRACKING_TRANSLATION_OFFSET,
            Constants.SWERVE.GAMEPIECE_TRACKING_TRANSLATION_TOLERANCE))
        & (hangTimer >= Constants.SWERVE.GAMEPIECE_TRACKING_LOST_TARGET_ALLOWED_LOOPS
            || limelightSub.validTargetExists());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.drive.drive(0, 0, 0);
    Robot.state.setDriveControlState(DriveControlState.FIELD_RELATIVE);
    System.out.println(interrupted);
  }
}