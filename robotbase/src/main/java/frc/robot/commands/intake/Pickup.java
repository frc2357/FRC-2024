package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.LEDs.LEDsSetColor;
import frc.robot.commands.rumble.RumbleDriverController;
import frc.robot.commands.state.SetNoteState;
import frc.robot.state.RobotState.NoteState;
import frc.robot.subsystems.LEDs;

public class Pickup extends SequentialCommandGroup {
  public Pickup() {
    super(
        new LEDsSetColor(Color.kWhite),

        // Run until we pickup note
        new IntakeRunUntilBeamState(INTAKE.PICKUP_SPEED_PERCENT_OUTPUT, true),
        new SetNoteState(NoteState.NOTE_IN_INTAKE),
        new InstantCommand(
            () -> {
              new RumbleDriverController().schedule();
            }),
        new LEDsSetColor(LEDs.GREEN));
  }
}
