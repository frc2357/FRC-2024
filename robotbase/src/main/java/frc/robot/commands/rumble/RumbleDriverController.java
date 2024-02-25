package frc.robot.commands.rumble;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class RumbleDriverController extends SequentialCommandGroup {
  public RumbleDriverController(double intensity, double seconds) {
    super(
        new InstantCommand(
            () -> {
              Robot.driverControls.setRumble(intensity);
            }),
        new WaitCommand(seconds),
        new InstantCommand(
            () -> {
              Robot.driverControls.setRumble(0);
            }));
  }
}
