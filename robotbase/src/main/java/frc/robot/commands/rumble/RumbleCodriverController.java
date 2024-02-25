package frc.robot.commands.rumble;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class RumbleCodriverController extends SequentialCommandGroup {
  public RumbleCodriverController(double intensity, double seconds) {
    super(
        new InstantCommand(
            () -> {
              Robot.codriverControls.setRumble(intensity);
            }),
        new WaitCommand(seconds),
        new InstantCommand(
            () -> {
              Robot.codriverControls.setRumble(0);
            }));
  }
}
