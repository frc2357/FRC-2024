package frc.robot.commands.source;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.Constants.PIVOT;
import frc.robot.commands.intake.IntakeReverseFeed;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPM;

public class SourceIntakeFromShooter extends ParallelDeadlineGroup {
  public SourceIntakeFromShooter() {
    super(
        new IntakeReverseFeed(),
        new ShooterSetRPM(Constants.SHOOTER.SOURCE_INTAKE_RPM),
        new PivotHoldAngle(PIVOT.INTAKE_FROM_SOURCE_ANGLE));
  }
}
