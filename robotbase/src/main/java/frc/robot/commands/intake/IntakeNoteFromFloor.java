package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.rumble.RumbleDriverController;
import frc.robot.commands.state.SetNoteState;
import frc.robot.state.RobotState.NoteState;

public class IntakeNoteFromFloor extends SequentialCommandGroup {
  public IntakeNoteFromFloor() {
    super(
        // Run until sensor sees the intake
        new IntakeRunUntilBeamState(INTAKE.PICKUP_SPEED_PERCENT_OUTPUT, true),
        new SetNoteState(NoteState.NOTE_IN_INTAKE),

        // Run note up against shooter rollers to compress it
        new ParallelDeadlineGroup(
            new WaitCommand(INTAKE.COMPRESS_NOTE_TIMEOUT),
            new RumbleDriverController(),
            new IntakeRun(INTAKE.COMPRESS_NOTE_SPEED_PERCENT_OUTPUT, true)),

        // Run the note back down until the sensor sees it
        new IntakeRunUntilBeamState(INTAKE.REVERSE_FEED_SPEED_PERCENT_OUTPUT, true),
        new IntakeStop(),
        new SetNoteState(NoteState.NOTE_STOWED));
  }
}
