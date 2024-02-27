package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.state.SetIntakeState;
import frc.robot.state.RobotState.IntakeState;

public class IntakeNoteFromFloor extends SequentialCommandGroup {
  public IntakeNoteFromFloor() {
    super(
        new IntakeRunUntilBeamState(INTAKE.PICKUP_SPEED_PERCENT_OUTPUT, true),
        new SetIntakeState(IntakeState.NOTE_IN_INTAKE),
        new IntakeRunUntilBeamState(INTAKE.SLOW_PICKUP_SPEED_PERCENT_OUTPUT, false),
        new SetIntakeState(IntakeState.NOTE_PAST_BEAM_BREAK),
        new IntakeAxis(() -> INTAKE.REVERSE_FEED_SPEED_PERCENT_OUTPUT)
            .withTimeout(INTAKE.FLOOR_INTAKE_REVERSE_TIMEOUT),
        new SetIntakeState(IntakeState.NOTE_STOWED));
  }
}
