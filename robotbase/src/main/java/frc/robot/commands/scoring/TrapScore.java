package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.commands.endAffector.EndAffectorSetSpeed;
import frc.robot.commands.endAffector.EndAffectorStop;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;

public class TrapScore extends SequentialCommandGroup {
  public TrapScore() {
    addCommands(
        new ExtensionArmMoveToRotations(EXTENSION_ARM.TRAP_SCORE_ROTATIONS),
        new EndAffectorSetSpeed(END_AFFECTOR.SCORE_SPEED_TRAP),
        new WaitCommand(END_AFFECTOR.SECONDS_TO_SCORE_TRAP),
        new EndAffectorStop(),
        new ExtensionArmMoveToRotations(EXTENSION_ARM.HOME_ROTATIONS));
  }
}
