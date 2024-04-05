package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.endAffector.EndAffectorStop;
import frc.robot.commands.extensionArm.ExtensionArmReturnToZero;
import frc.robot.state.RobotState.NoteState;

public class AmpSequenceConditional extends ConditionalCommand {
  public AmpSequenceConditional() {
    super(
        new AmpPrepose(),
        new AmpScore().handleInterrupt(() -> {new ParallelCommandGroup(
          new ExtensionArmReturnToZero(),
          new EndAffectorStop()
        ).schedule();}),
        () -> !Robot.state.isNote(NoteState.END_AFFECTOR_PRELOAD));
  }
}
