package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.state.RobotState;

public class DriverAmpScore extends SequentialCommandGroup {
  public DriverAmpScore() {
    super(
        new ConditionalCommand(
            new ConditionalCommand(
                new ConditionalCommand(
                    new AmpScore(),
                    new AmpPrepose(),
                    () -> Robot.state.isInState(RobotState.State.AMP_PRE_POSE)),

                new NotePreload(),

                () -> Robot.state.isInState(RobotState.State.NOTE_PRELOAD)),

            new InstantCommand(),
            () -> Robot.state.hasNote()));
  }
}
