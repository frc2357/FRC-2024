package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.Constants.SCORING;
import frc.robot.commands.endAffector.EndAffectorSetSpeed;
import frc.robot.commands.endAffector.EndAffectorStop;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;

public class AmpScore extends SequentialCommandGroup {
  public AmpScore() {
    super(
        new EndAffectorSetSpeed(END_AFFECTOR.SCORE_SPEED_AMP),
        new WaitCommand(SCORING.SECONDS_AMP_SCORE),
        new EndAffectorStop(),
        new ExtensionArmMoveToRotations(EXTENSION_ARM.HOME_ROTATIONS));
  }
}
