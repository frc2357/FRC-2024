package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.LEDs.LEDsSetHasNote;
import frc.robot.commands.LEDs.LEDsSetIntaking;
import frc.robot.commands.intake.IntakeRunUntilBeamState;

public class AutoPickup extends SequentialCommandGroup {
  public AutoPickup() {
    super(
        new LEDsSetIntaking(),

        // Run until we pickup note
        new IntakeRunUntilBeamState(INTAKE.PICKUP_SPEED_PERCENT_OUTPUT, true),
        new LEDsSetHasNote());
  }
}
