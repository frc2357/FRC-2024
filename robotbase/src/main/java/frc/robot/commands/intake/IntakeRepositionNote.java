package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE;

public class IntakeRepositionNote extends SequentialCommandGroup {
  public IntakeRepositionNote() {
    super(
        // Run slow until past beam break
        new IntakeRunUntilBeamState(INTAKE.SLOW_PICKUP_SPEED_PERCENT_OUTPUT, false),

        // Run backwards to keep out of shooter
        new IntakeRun(INTAKE.REVERSE_FEED_SPEED_PERCENT_OUTPUT)
            .withTimeout(INTAKE.FLOOR_INTAKE_REVERSE_TIMEOUT));
  }
}
