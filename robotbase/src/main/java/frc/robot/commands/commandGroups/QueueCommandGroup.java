// THIS COMMAND DOES NOT WORK. DO NOT USE THIS COMMAND YET. NO. BAD.
package frc.robot.commands.commandGroups;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.List;

public class QueueCommandGroup extends Command {
  private final List<Command> m_commands = new ArrayList<>();
  private int m_currentCommandIndex = -1;
  private int m_numQueued = -1;
  private boolean m_runWhenDisabled = true;
  private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

  /**
   * Creates a new QueueCommandGroup. The given commands will be run sequentially, each time the
   * command is ran, the next command in the command list is queued.
   *
   * @param commands the commands to include in this composition.
   */
  public QueueCommandGroup(Command... commands) {
    addCommands(commands);
  }

  /**
   * Adds the given commands to the group.
   *
   * @param commands Commands to add, in order of execution.
   */
  public final void addCommands(Command... commands) {
    if (m_currentCommandIndex != -1) {
      throw new IllegalStateException(
          "Commands cannot be added to a composition while it's running");
    }

    CommandScheduler.getInstance().registerComposedCommands(commands);

    for (Command command : commands) {
      m_commands.add(command);
      super.addRequirements(command.getRequirements());
      m_runWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        m_interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  @Override
  public final void initialize() {
    if (m_commands.isEmpty()) {
      return;
    }
    Command currentCommand = m_commands.get(m_currentCommandIndex);

    m_numQueued += m_numQueued == -1 ? 2 : 1;

    if (m_numQueued > m_commands.size()) {
      m_numQueued = -1;
    }

    if (!currentCommand.isScheduled()) {
      currentCommand.schedule();
    }
  }

  @Override
  public final void execute() {
    if (m_commands.isEmpty()) {
      return;
    }

    Command currentCommand = m_commands.get(m_currentCommandIndex);

    if (!currentCommand.isScheduled()) {
      m_currentCommandIndex++;
      if (m_currentCommandIndex < m_numQueued) {
        m_commands.get(m_currentCommandIndex).schedule();
      }
    }
  }

  @Override
  public final void end(boolean interrupted) {
    if (!m_commands.isEmpty() && m_numQueued == -1) {
      m_commands.get(m_currentCommandIndex);
    }
  }

  @Override
  public final boolean isFinished() {
    return m_currentCommandIndex == m_numQueued + 1 || m_numQueued == -1;
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_runWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_interruptBehavior;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addIntegerProperty("index", () -> m_currentCommandIndex, null);
  }
}
