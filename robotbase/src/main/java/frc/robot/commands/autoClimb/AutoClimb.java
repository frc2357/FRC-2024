package frc.robot.commands.autoClimb;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.commands.climber.ClimberLevelClimb;
import frc.robot.commands.climber.ClimberRotatePastRotations;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.scoring.NotePreload;

public class AutoClimb extends SequentialCommandGroup {
  public AutoClimb() {
    super(
        // Lineup on apriltag + raise arms to specific point below chain

        // Drive towards stage

        // Raise arms to 10 degrees past vertical

        // Drive away from stage until hooks catch

        // Lower hooks + slowly drive towards stage until arms at specific point for
        // Extension arm to extend
        new ClimberRotatePastRotations(
            CLIMBER.ROTATE_PAST_EXTENSION_SPEED, CLIMBER.NOTE_HANDOFF_MAX_ROTATIONS),

        // Do note handoff (note should be as high in the end affector as possible)
        new NotePreload(),

        // Extend arm to final extension
        new ExtensionArmMoveToRotations(EXTENSION_ARM.TRAP_SCORE_ROTATIONS),

        // Slowly climb until we see a sharp increase in gyro angle
        new ParallelDeadlineGroup(new WaitForEndAffectorDrop(), new ClimberLevelClimb()));
  }
}
