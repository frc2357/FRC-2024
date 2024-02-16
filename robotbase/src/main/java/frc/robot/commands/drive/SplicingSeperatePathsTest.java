package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SplicingSeperatePathsTest extends SequentialCommandGroup {
  public SplicingSeperatePathsTest() {
    addCommands(
        new DriveChoreoPath("SplicingTest1", "Sep Test", true),
        new ParallelDeadlineGroup(new WaitCommand(1), new DriveToGamepeice()),
        new DriveChoreoPath("SplicingTest2", "Sep Test", false));
  }

  @Override
  public String toString() {
    return "Com Splice Test Group - Seperate Paths + Drive To GamePeice";
  }
}
