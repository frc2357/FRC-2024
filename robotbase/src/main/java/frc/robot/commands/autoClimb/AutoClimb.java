package frc.robot.commands.autoClimb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        new ParallelCommandGroup(
            // Lineup on apriltag. Far enough away to see the apriltag
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_PREPOSE_SPEED, CLIMBER.PREPOSE_ROTATIONS)),

        // Drive towards stage

        // Raise arms to 10 degrees past vertical
        new ClimberRotatePastRotations(
            CLIMBER.ROTATE_PAST_TEN_DEGREES_SPEED, CLIMBER.TEN_DEGREES_ROTATIONS),

        // Drive away from stage until hooks catch (drive at set speed (slow) until the
        // speed drops below a certain threshold)

        // Lower hooks + slowly drive towards stage until arms at specific point for
        // Extension arm to extend
        new ParallelDeadlineGroup(
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_EXTENSION_SPEED, CLIMBER.NOTE_HANDOFF_MAX_ROTATIONS)
            // Drive at set speed
            ),

        // Do note handoff (note should be as high in the end affector as possible)
        new NotePreload(),

        // Extend arm to final extension
        new ExtensionArmMoveToRotations(EXTENSION_ARM.TRAP_SCORE_ROTATIONS),

        // Slowly climb until we see a sharp increase in gyro angle
        new ParallelDeadlineGroup(new WaitForEndAffectorDrop(), new ClimberLevelClimb()));
  }
}
