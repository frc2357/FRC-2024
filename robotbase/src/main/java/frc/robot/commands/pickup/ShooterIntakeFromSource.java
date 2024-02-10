package frc.robot.commands.pickup;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.intake.IntakeNoteFromSource;
import frc.robot.commands.pivot.ShooterPivotIntakeFromSource;

public class ShooterIntakeFromSource extends ParallelRaceGroup {

  public ShooterIntakeFromSource() {
    addCommands(new IntakeNoteFromSource());
    addCommands(new ShooterIntakeFromSource());
    addCommands(new ShooterPivotIntakeFromSource());
  }
}
