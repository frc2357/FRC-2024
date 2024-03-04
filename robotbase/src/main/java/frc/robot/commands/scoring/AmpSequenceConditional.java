package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.state.RobotState.NoteState;

public class AmpSequenceConditional extends ConditionalCommand {
  public AmpSequenceConditional() {
    super(
        new AmpPrepose(),
        new AmpScore(),
        () -> !Robot.state.isNote(NoteState.END_AFFECTOR_PRELOAD));
  }
}
