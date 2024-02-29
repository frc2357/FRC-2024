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
import frc.robot.commands.endAffector.EndAffectorStop;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPMs;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.state.SetNoteState;
import frc.robot.state.RobotState.NoteState;

public class AmpPrepose extends ParallelDeadlineGroup {
  public AmpPrepose() {
    super(
        new SequentialCommandGroup(
            // Note Preload
            new ExtensionArmMoveToRotations(EXTENSION_ARM.NOTE_STOW_ROTATIONS),

            // Run end affector, shooter, and intake to load note
            new ParallelDeadlineGroup(
                new WaitCommand(SCORING.SECONDS_PRELOAD_NOTE),
                new SequentialCommandGroup(new WaitCommand(0.25), new IntakeFeedToShooter()),
                new EndAffectorSetSpeed(END_AFFECTOR.PRELOAD_SPEED),
                new ShooterSetRPMs(
                    SHOOTER.TOP_MOTOR_FEED_END_AFFECTOR_RPMS,
                    SHOOTER.BOTTOM_MOTOR_FEED_END_AFFECTOR_RPMS)),

            // Stop motors
            new ParallelCommandGroup(new IntakeStop(), new ShooterStop(), new EndAffectorStop()),

            // Arm Prepose
            new ExtensionArmMoveToRotations(EXTENSION_ARM.AMP_PREPOSE_ROTATIONS),
            new SetNoteState(NoteState.END_AFFECTOR_PRELOAD)),

        // Hold until end of above command
        new PivotHoldAngle(PIVOT.END_AFFECTOR_PRELOAD_ANGLE));
  }
}
