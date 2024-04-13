package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.LEDs.LEDsSetHasNote;
import frc.robot.commands.LEDs.LEDsSetIntaking;
import frc.robot.commands.rumble.RumbleDriverController;

public class Pickup extends SequentialCommandGroup {
  public Pickup() {
    super(
        new LEDsSetIntaking(),

        // Run until we pickup note
        new IntakeRunUntilBeamState(INTAKE.PICKUP_SPEED_PERCENT_OUTPUT, true),
        new InstantCommand(
            () -> {
              new RumbleDriverController().schedule();
            }),
        new LEDsSetHasNote());
  }
}
