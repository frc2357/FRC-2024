package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.Constants.PIVOT;
import frc.robot.Constants.SHOOTER;
import frc.robot.commands.endAffector.EndAffectorRunPastTopEdge;
import frc.robot.commands.endAffector.EndAffectorStop;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.state.SetNoteState;
import frc.robot.state.RobotState.NoteState;
import frc.robot.util.TurnOnProximitySensor;

public class AmpPrepose extends SequentialCommandGroup {
  public AmpPrepose() {
    super(
        new ParallelCommandGroup(
            new TurnOnProximitySensor(),
            new ExtensionArmMoveToRotations(EXTENSION_ARM.NOTE_STOW_ROTATIONS)),
        // Run end affector, shooter, and intake to load note
        new ParallelDeadlineGroup(
            new EndAffectorRunPastTopEdge(true),
            new IntakeFeedToShooter().beforeStarting(new WaitCommand(0.2)),
            new ShooterSetRPM(SHOOTER.FEED_END_AFFECTOR_RPM),
            new PivotHoldAngle(PIVOT.END_AFFECTOR_PRELOAD_ANGLE)),

        // Arm Prepose
        new ParallelCommandGroup(
          new ExtensionArmMoveToRotations(EXTENSION_ARM.AMP_PREPOSE_ROTATIONS),
          new SequentialCommandGroup(
            new WaitCommand(0.25),
            new EndAffectorStop()
          )
        ),
        new SetNoteState(NoteState.END_AFFECTOR_PRELOAD));
  }
}
