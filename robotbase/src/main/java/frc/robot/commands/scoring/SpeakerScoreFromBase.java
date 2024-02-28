package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PIVOT;
import frc.robot.Constants.SHOOTER;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPMs;
import frc.robot.commands.shooter.ShooterWaitForRPMs;
import frc.robot.commands.state.SetNoteState;
import frc.robot.state.RobotState.NoteState;

public class SpeakerScoreFromBase extends ParallelDeadlineGroup {
  public SpeakerScoreFromBase() {
    super(
        new ParallelCommandGroup(
            new ShooterSetRPMs(
                SHOOTER.TOP_MOTOR_SUBWOOFER_SHOT_RPMS, SHOOTER.BOTTOM_MOTOR_SUBWOOFER_SHOT_RPMS),
            new SequentialCommandGroup(
                new ShooterWaitForRPMs(),
                new SetNoteState(NoteState.EMPTY),
                new IntakeFeedToShooter())),

        // Hold until above sequence is done
        new PivotHoldAngle(PIVOT.SUBWOOFER_SHOT_ANGLE));
  }
}
