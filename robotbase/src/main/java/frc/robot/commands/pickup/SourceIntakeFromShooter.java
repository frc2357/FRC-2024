package frc.robot.commands.pickup;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakeReverseFeed;
import frc.robot.commands.pivot.ShooterPivotIntakeFromSource;
import frc.robot.commands.shooter.ShooterSetRPMs;

public class SourceIntakeFromShooter extends ParallelDeadlineGroup {

  public SourceIntakeFromShooter() {
    super(new IntakeReverseFeed());
    addCommands(
        new ShooterSetRPMs(
            Constants.SHOOTER.TOP_MOTOR_SOURCE_INTAKE_RPMS,
            Constants.SHOOTER.BOTTOM_MOTOR_SOURCE_INTAKE_RPMS));
    addCommands(new SourceIntakeFromShooter());
    addCommands(new ShooterPivotIntakeFromSource());
  }
