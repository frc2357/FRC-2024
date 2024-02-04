package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ChoreoTrajectoryCommand;
import frc.robot.commands.DriveToGamepeiceCommand;

public class SplicingSeperatePathsTestCommand extends SequentialCommandGroup {
  public SplicingSeperatePathsTestCommand() {
    addCommands(
        new ChoreoTrajectoryCommand("SplicingTest1", "Sep Test", true),
        new ParallelDeadlineGroup(new WaitCommand(1), new DriveToGamepeiceCommand()),
        new ChoreoTrajectoryCommand("SplicingTest2", "Sep Test", false));
  }

  @Override
  public String toString() {
    return "Com Splice Test Group - Seperate Paths + Drive To GamePeice";
  }
}
