package frc.robot.commands.pickup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.intake.IntakeRepositionNote;
import frc.robot.subsystems.LEDs;

public class Pickup extends SequentialCommandGroup {
  public Pickup() {
    super(
        new Pickup(),
        new IntakeRepositionNote()
            .handleInterrupt(() -> Robot.leds.setColor(LEDs.MELTDOWN_ORANGE)));
  }
}
