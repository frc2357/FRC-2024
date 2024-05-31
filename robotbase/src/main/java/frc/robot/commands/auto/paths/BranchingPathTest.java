package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.PathNode;
import frc.robot.commands.drive.DriveChoreoPath;

public class BranchingPathTest extends SequentialCommandGroup {
  public BranchingPathTest() {
    super(
        new DriveChoreoPath("ShortSquareTestPath1"),
        new PathNode(
            new DriveChoreoPath("ShortSquareTestPath2"),
            new InstantCommand(() -> System.out.println("[Branching Path Test] Node was false")),
            () -> true,
            "Branching Path Test Node"));
  }
}
