package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.rumble.RumbleDriverController;
import frc.robot.commands.state.SetNoteState;
import frc.robot.state.RobotState.NoteState;

public class IntakeNoteFromFloor extends SequentialCommandGroup {
  public IntakeNoteFromFloor() {
    super(
        new IntakeRunUntilBeamState(INTAKE.PICKUP_SPEED_PERCENT_OUTPUT, true),
        new SetNoteState(NoteState.NOTE_IN_INTAKE),
        new ParallelDeadlineGroup(
            new IntakeRunUntilBeamState(INTAKE.SLOW_PICKUP_SPEED_PERCENT_OUTPUT, false),
            new RumbleDriverController()),
        new SetNoteState(NoteState.NOTE_PAST_BEAM_BREAK),
        // new ParallelDeadlineGroup(
        // new WaitCommand(INTAKE.FLOOR_INTAKE_REVERSE_TIMEOUT),
        // new InstantCommand(() ->
        // Robot.intake.set(INTAKE.REVERSE_FEED_SPEED_PERCENT_OUTPUT))),
        new IntakeStop(),
        new SetNoteState(NoteState.NOTE_STOWED));
  }
}
