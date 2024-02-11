package frc.robot.commands.pickup;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.intake.IntakeReverseFeed;
import frc.robot.commands.pivot.ShooterPivotIntakeFromSource;

public class SourceIntakeFromShooter extends ParallelDeadlineGroup {

  public SourceIntakeFromShooter() {
    super(new IntakeReverseFeed());
    addCommands(new SourceIntakeFromShooter());
    addCommands(new ShooterPivotIntakeFromSource());
  }
}
