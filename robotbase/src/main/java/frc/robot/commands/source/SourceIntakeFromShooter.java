package frc.robot.commands.source;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.Constants.PIVOT;
import frc.robot.Robot;
import frc.robot.commands.intake.IntakeReverseFeed;
import frc.robot.commands.pivot.PivotSetRotation;
import frc.robot.commands.shooter.ShooterSetRPMs;
import frc.robot.state.RobotState.NoteState;

public class SourceIntakeFromShooter extends ParallelDeadlineGroup {
  public SourceIntakeFromShooter() {
    super(
        new IntakeReverseFeed()
            .finallyDo(
                (boolean interrupted) -> {
                  if (!interrupted)
                    Robot.state.setNoteState(NoteState.NOTE_STOWED);
                }));
    addCommands(
        new ShooterSetRPMs(
            Constants.SHOOTER.TOP_MOTOR_SOURCE_INTAKE_RPMS,
            Constants.SHOOTER.BOTTOM_MOTOR_SOURCE_INTAKE_RPMS));
    addCommands(new SourceIntakeFromShooter());
    addCommands(new PivotSetRotation(PIVOT.INTAKE_FROM_SOURCE_ROTATION));
  }
}
