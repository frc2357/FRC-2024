package frc.robot.commands.source;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PIVOT;
import frc.robot.Constants.SHOOTER;
import frc.robot.commands.intake.IntakeReverseFeed;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.intake.WaitForBeamBreak;
import frc.robot.commands.pivot.PivotSetRotation;
import frc.robot.commands.shooter.ShooterSetRPMs;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.state.RobotState.NoteState;

public class SourceIntakeFromShooter extends SequentialCommandGroup {
  public SourceIntakeFromShooter() {
    super(
        new ParallelDeadlineGroup(
            new WaitForBeamBreak(),
            new IntakeReverseFeed(),
            new ShooterSetRPMs(
                SHOOTER.TOP_MOTOR_SOURCE_INTAKE_RPMS, SHOOTER.BOTTOM_MOTOR_SOURCE_INTAKE_RPMS),
            new PivotSetRotation(PIVOT.INTAKE_FROM_SOURCE_ROTATION)),
        new ParallelCommandGroup(new IntakeStop(), new ShooterStop()));
  }
}
