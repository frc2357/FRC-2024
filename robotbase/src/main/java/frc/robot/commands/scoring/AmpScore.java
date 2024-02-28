package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.Constants.SCORING;
import frc.robot.commands.endAffector.EndAffectorSetSpeed;
import frc.robot.commands.endAffector.EndAffectorStop;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.state.SetNoteState;
import frc.robot.state.RobotState.NoteState;

public class AmpScore extends SequentialCommandGroup {
  public AmpScore() {
    super(
        new ExtensionArmMoveToRotations(EXTENSION_ARM.AMP_SCORE_ROTATIONS),
        new EndAffectorSetSpeed(END_AFFECTOR.SCORE_SPEED_AMP),
        new WaitCommand(SCORING.SECONDS_AMP_SCORE),
        new EndAffectorStop(),
        new ExtensionArmMoveToRotations(EXTENSION_ARM.HOME_ROTATIONS),
        new SetNoteState(NoteState.EMPTY));
  }
}
