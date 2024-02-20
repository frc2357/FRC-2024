package frc.robot.commands.scoring;

import frc.robot.commands.commandGroups.QueueCommandGroup;

public class DriverAmpScore extends QueueCommandGroup {

  public DriverAmpScore() {
    super(
        new NotePreload(),
        new AmpPrepose(),
        new AmpScore());
  }
}
