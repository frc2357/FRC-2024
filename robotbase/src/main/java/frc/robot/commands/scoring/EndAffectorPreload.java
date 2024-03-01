package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.Constants.PIVOT;
import frc.robot.Constants.SCORING;
import frc.robot.Constants.SHOOTER;
import frc.robot.commands.endAffector.EndAffectorSetSpeed;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPM;

public class EndAffectorPreload extends SequentialCommandGroup {
  public EndAffectorPreload() {
    super(
        new ParallelCommandGroup(
            new ExtensionArmMoveToRotations(EXTENSION_ARM.NOTE_STOW_ROTATIONS),
            new PivotHoldAngle(PIVOT.END_AFFECTOR_PRELOAD_ANGLE)),
        new ParallelDeadlineGroup(
            new WaitCommand(SCORING.SECONDS_PRELOAD_NOTE),
            new SequentialCommandGroup(new WaitCommand(0.25), new IntakeFeedToShooter()),
            new EndAffectorSetSpeed(END_AFFECTOR.PRELOAD_SPEED),
            new ShooterSetRPM(SHOOTER.FEED_END_AFFECTOR_RPM)));
  }
}
