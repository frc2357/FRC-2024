package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.Constants.PIVOT;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.state.SetNoteState;
import frc.robot.state.RobotState.NoteState;

public class EndAffectorPrepose extends ParallelDeadlineGroup {
  public EndAffectorPrepose() {
    super(
        new SequentialCommandGroup(
            new EndAffectorPreload(),

            // Arm Prepose
            new ExtensionArmMoveToRotations(EXTENSION_ARM.AMP_PREPOSE_ROTATIONS),
            new SetNoteState(NoteState.END_AFFECTOR_PRELOAD)),

        // Hold until end of above command
        new PivotHoldAngle(PIVOT.END_AFFECTOR_PRELOAD_ANGLE));
  }
}
