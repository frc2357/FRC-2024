package frc.robot.commands.pickup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DrivePickup;
import frc.robot.commands.intake.IntakeNoteFromFloor;
import frc.robot.commands.intake.IntakeRepositionNote;
import frc.robot.subsystems.LEDs;

public class VisionPickup extends SequentialCommandGroup {
  public VisionPickup() {
    super(
        new ParallelDeadlineGroup(
            new IntakeNoteFromFloor(),
            new SequentialCommandGroup(new WaitCommand(0.25), new DrivePickup())),
        new ParallelCommandGroup(
            new DefaultDrive(),
            new IntakeRepositionNote()
                .handleInterrupt(() -> Robot.leds.setColor(LEDs.MELTDOWN_ORANGE))));
  }
}
