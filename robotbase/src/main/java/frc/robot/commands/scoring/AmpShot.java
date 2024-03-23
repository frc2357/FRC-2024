package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.Constants.SCORING;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.extensionArm.ExtensionArmReturnToZero;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.util.PressToContinue;

public class AmpShot extends ParallelDeadlineGroup {
  public AmpShot(Trigger continueButton) {
    super(
      new SequentialCommandGroup(
        new ExtensionArmMoveToRotations(EXTENSION_ARM.AMP_SHOT_PREPOSE_ROTATIONS),

        new PressToContinue(continueButton),

        new IntakeFeedToShooter().withTimeout(0.5),
        new ExtensionArmReturnToZero()
      ),
      new ShooterSetRPM(SCORING.AMP_SHOT_SHOOTER_RPMS),
      new PivotHoldAngle(SCORING.AMP_SHOT_PIVOT_ANGLE)
    );
  }
}
