package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.commands.pivot.PivotSetRotation;
import frc.robot.commands.shooter.ShooterSetRPMs;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.state.SetRobotStateCommand;
import frc.robot.state.RobotState;

public class NotePreload extends SequentialCommandGroup {

  public NotePreload() {
    super(
        new ParallelCommandGroup(
            // Extend arm to position to receive from shooter
            new ExtensionArmMoveToRotations(EXTENSION_ARM.NOTE_STOW_ROTATIONS),
            // Rotate pivot to feed into end affector
            new PivotSetRotation(PIVOT.FEED_TO_END_AFFECTOR_LOCATION)),

        // Run end affector, shooter, and intake to load note
        new ParallelDeadlineGroup(
            new WaitCommand(SCORING.SECONDS_PRELOAD_NOTE),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new IntakeFeedToShooter()),
            new ShooterSetRPMs(
                SHOOTER.TOP_MOTOR_FEED_END_AFFECTOR_RPMS,
                SHOOTER.BOTTOM_MOTOR_FEED_END_AFFECTOR_RPMS),
            new EndAffectorSetSpeed(END_AFFECTOR.PRELOAD_SPEED)),

        // Stop motors
        new ParallelCommandGroup(new IntakeStop(), new ShooterStop(), new EndAffectorStop()),
        new SetRobotStateCommand(RobotState.State.NOTE_PRELOAD));

    // TODO: logic to position note for amp or trap
  }
}
