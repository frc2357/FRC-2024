package frc.robot.commands.extensionArm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EXTENSION_ARM;

public class ExtensionArmReturnToZero extends SequentialCommandGroup {
  public ExtensionArmReturnToZero() {
    super(
        new ExtensionArmMoveToRotations(EXTENSION_ARM.READY_TO_ZERO_ROTATIONS),
        new ExtensionArmZero());
  }
}
