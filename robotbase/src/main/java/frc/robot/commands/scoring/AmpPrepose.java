package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;

public class AmpPrepose extends SequentialCommandGroup {
  public AmpPrepose() {
    super(new ExtensionArmMoveToRotations(EXTENSION_ARM.AMP_SCORE_ROTATIONS));
  }
}
