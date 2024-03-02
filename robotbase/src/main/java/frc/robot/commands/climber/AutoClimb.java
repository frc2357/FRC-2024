package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.drive.DriveAtSpeed;
import frc.robot.commands.drive.DriveToApriltag;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.scoring.EndAffectorPrepose;

public class AutoClimb extends SequentialCommandGroup {
  public AutoClimb() {
    super(
        // Lineup on apriltag + raise arms to specific point below chain
        new ParallelCommandGroup(
            new DriveToApriltag(SWERVE.CLIMB_TY_SETPOINT, 0, () -> 0.0, null),
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_PREPOSE_SPEED, CLIMBER.PREPOSE_ROTATIONS)),
        new DriveAtSpeed(
            0,
            SWERVE.DRIVE_TOWARDS_STAGE_APRILTAG_Y_DISTANCE
                / SWERVE.DRIVE_TOWARDS_STAGE_APRILTAG_SECONDS,
            SWERVE.DRIVE_TOWARDS_STAGE_APRILTAG_SECONDS),
        new ParallelDeadlineGroup(
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_TEN_DEGREES_SPEED, CLIMBER.TEN_DEGREES_ROTATIONS),
            new DriveAtSpeed(
                0,
                SWERVE.DRIVE_TOWARDS_CHAIN_Y_SPEED,
                SWERVE.DRIVE_TOWARDS_STAGE_APRILTAG_SECONDS)),
        new ClimberRotatePastRotations(
            CLIMBER.ROTATE_PAST_EXTENSION_SPEED, CLIMBER.NOTE_HANDOFF_MAX_ROTATIONS),
        new EndAffectorPrepose(),
        new ExtensionArmMoveToRotations(EXTENSION_ARM.TRAP_SCORE_ROTATIONS));
  }
}
