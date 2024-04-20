package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.LEDs.LEDsSetHasNote;
import frc.robot.commands.LEDs.LEDsSetIntaking;
import frc.robot.commands.rumble.RumbleDriverController;
import frc.robot.commands.shooter.ShooterSetRPM;

public class Pickup extends ParallelDeadlineGroup {
  public Pickup() {
    super(
        new SequentialCommandGroup(
            new LEDsSetIntaking(),

            // Run until we pickup note
            new IntakeRunUntilBeamState(INTAKE.PICKUP_SPEED_PERCENT_OUTPUT, true),
            new InstantCommand(
                () -> {
                  new RumbleDriverController().schedule();
                }),
            new LEDsSetHasNote()),
        new ShooterSetRPM(0));
  }
}
