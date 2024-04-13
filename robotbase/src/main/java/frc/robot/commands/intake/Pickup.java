package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.LEDs.LEDsSetHasNote;
import frc.robot.commands.LEDs.LEDsSetIntaking;
import frc.robot.commands.rumble.RumbleDriverController;
import frc.robot.commands.state.SetNoteState;
import frc.robot.state.RobotState.NoteState;

public class Pickup extends SequentialCommandGroup {
  public Pickup() {
    super(
        new LEDsSetIntaking(),

        // Run until we pickup note
        new IntakeRunUntilBeamState(INTAKE.PICKUP_SPEED_PERCENT_OUTPUT, true),
        new SetNoteState(NoteState.NOTE_IN_INTAKE),
        new InstantCommand(
            () -> {
              new RumbleDriverController().schedule();
            }),
        new LEDsSetHasNote());
  }
}
