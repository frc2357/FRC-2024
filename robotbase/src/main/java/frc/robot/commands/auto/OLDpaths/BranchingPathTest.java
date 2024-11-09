package frc.robot.commands.auto.OLDpaths;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.PathNode;

public class BranchingPathTest extends SequentialCommandGroup {
  public BranchingPathTest() {
    super(
        new PrintCommand("[Branching Path Test] Starting path node"),
        new PathNode(
            new PrintCommand("[Branching Path Test] Node was true"),
            new PrintCommand("[Branching Path Test] Node was false"),
            () -> false,
            "Branching Path Test Node"));
  }

  @Override
  public String toString() {
    return "branching path print test";
  }
}
