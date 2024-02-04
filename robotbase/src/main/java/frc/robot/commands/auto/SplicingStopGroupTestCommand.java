package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ChoreoTrajectoryCommand;
import frc.robot.commands.DriveToGamepeiceCommand;

public class SplicingStopGroupTestCommand extends SequentialCommandGroup {

  public SplicingStopGroupTestCommand() {
    addCommands(
        new ChoreoTrajectoryCommand("BoxTest.1", "BoxTest", true),
        new ParallelDeadlineGroup(new WaitCommand(1), new DriveToGamepeiceCommand()),
        new ChoreoTrajectoryCommand("BoxTest.2", "BoxTest", false),
        new ChoreoTrajectoryCommand("BoxTest.3", "BoxTest", false),
        new ChoreoTrajectoryCommand("BoxTest.4", "BoxTest", false));
  }

  @Override
  public String toString() {
    return "Command Splicing test group - BoxTest + DriveToGamePeice";
  }
}
