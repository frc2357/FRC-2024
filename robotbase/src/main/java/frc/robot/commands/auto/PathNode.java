package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.util.function.BooleanSupplier;

public class PathNode extends ConditionalCommand {
  private final BooleanSupplier m_condition;
  private final String m_nodeName;

  /**
   * @param onTrue The command that is passed back if the condition is true
   * @param onFalse The command that is passed back if the condition is false
   * @param condition The boolean condition that determines which branch of the path is used
   * @param nodeName Name of the node used to print out logging things.
   */
  public PathNode(Command onTrue, Command onFalse, BooleanSupplier condition, String nodeName) {
    super(onTrue, onFalse, condition);
    m_condition = condition;
    m_nodeName = nodeName;
  }

  @Override
  public void initialize() { // overiding so we can add what we want to when its first ran
    System.out.println( // mostly just logging
        "["
            + m_nodeName
            + "] Condition when node's initialize was called: "
            + m_condition.getAsBoolean());
    super.initialize(); // then we run the normal conditional command initialize
  }
}
