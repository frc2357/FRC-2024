package frc.robot.commands.source;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.endAffector.EndAffectorIntakeFromSource;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;

public class SourceIntakeFromEndAffector extends SequentialCommandGroup {

  public SourceIntakeFromEndAffector() {
    addCommands(
        new ExtensionArmMoveToRotations(EXTENSION_ARM.NOTE_STOW_ROTATIONS),
        new EndAffectorIntakeFromSource());
  }
}
