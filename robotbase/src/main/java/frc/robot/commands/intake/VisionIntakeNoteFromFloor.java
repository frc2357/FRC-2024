package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.LEDs.LEDsSetColor;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.TargetLockOnNote;
import frc.robot.commands.rumble.RumbleDriverController;
import frc.robot.commands.state.SetNoteState;
import frc.robot.state.RobotState.NoteState;
import frc.robot.subsystems.LEDs;

public class VisionIntakeNoteFromFloor extends SequentialCommandGroup {
  public VisionIntakeNoteFromFloor() {
    super(
        new LEDsSetColor(Color.kWhite),

        // Run until we pickup note
        new ParallelDeadlineGroup(
            new IntakeRunUntilBeamState(INTAKE.PICKUP_SPEED_PERCENT_OUTPUT, true),
            new TargetLockOnNote()),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new SetNoteState(NoteState.NOTE_IN_INTAKE),
                new InstantCommand(
                    () -> {
                      new RumbleDriverController().schedule();
                    }),
                new LEDsSetColor(LEDs.GREEN),

                // Run slow until past beam break
                new IntakeRunUntilBeamState(INTAKE.SLOW_PICKUP_SPEED_PERCENT_OUTPUT, false),
                new SetNoteState(NoteState.NOTE_PAST_BEAM_BREAK),

                // Run backwards to keep out of shooter
                new IntakeRun(INTAKE.REVERSE_FEED_SPEED_PERCENT_OUTPUT)
                    .withTimeout(INTAKE.FLOOR_INTAKE_REVERSE_TIMEOUT),
                new IntakeStop(),
                new SetNoteState(NoteState.NOTE_STOWED)),
            new DefaultDrive()));
  }
}
