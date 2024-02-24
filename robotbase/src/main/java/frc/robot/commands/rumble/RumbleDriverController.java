package frc.robot.commands.rumble;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;

public class RumbleDriverController extends SequentialCommandGroup {
  public RumbleDriverController() {
    super(
        new InstantCommand(
            () -> {
              Robot.driverControls.setRumble(CONTROLLER.DRIVE_RUMBLE_INTENSITY);
            }),
        new WaitCommand(CONTROLLER.DRIVE_RUMBLE_SECONDS),
        new InstantCommand(
            () -> {
              Robot.driverControls.setRumble(0);
            }));
  }
}