package frc.robot.commands.rumble;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;

public class RumbleCodriverController extends SequentialCommandGroup {
  public RumbleCodriverController() {
    super(
        new InstantCommand(
            () -> {
              Robot.codriverControls.setRumble(CONTROLLER.CODRIVE_RUMBLE_INTENSITY);
            }),
        new WaitCommand(CONTROLLER.CODRIVE_RUMBLE_SECONDS),
        new InstantCommand(
            () -> {
              Robot.codriverControls.setRumble(0);
            }));
  }
}
