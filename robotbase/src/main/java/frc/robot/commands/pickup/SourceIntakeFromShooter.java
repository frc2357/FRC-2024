package frc.robot.commands.pickup;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakeReverseFeed;
import frc.robot.commands.pivot.PivotSetRotation;

public class SourceIntakeFromShooter extends ParallelDeadlineGroup {

  public SourceIntakeFromShooter() {
    super(new IntakeReverseFeed());
    addCommands(new SourceIntakeFromShooter());
    addCommands(new PivotSetRotation(Constants.PIVOT.INTAKE_FROM_SOURCE_ROTATION));
  }
}
