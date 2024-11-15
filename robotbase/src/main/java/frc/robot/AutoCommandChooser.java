package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.util.VariableWaitCommand;
import java.util.HashMap;

public class AutoCommandChooser {
  private String[] m_autoNames;
  private SendableChooser<String> m_chooser;
  private SelectCommand m_selectCommand;
  private Command m_autoCommand;

  private String m_waitCommandKey = "wait";

  public AutoCommandChooser() {
    Command[] autoCommands = {
      // new SourceSide4Note(),
      // new SourceSide3NoteNoStage(),
      // new AmpSide4Note(),
      // new AmpSide4NoteWallFirst(),
      // new Close3AndRUN(),
      // new Close3AndMiddleCenterline(),
      // new AntiCitrus(),
      // new AmpSideBranchingPath(),
      // new SourceSideBranchingPath(), // These will not work until adapted to 2025 code. which i dont want to do.
    };

    HashMap<String, Command> commandMap = new HashMap<String, Command>(autoCommands.length + 1);
    m_autoNames = new String[autoCommands.length + 1];

    m_autoNames[0] = "None";
    commandMap.put("None", new WaitCommand(0));
    for (int i = 0; i < autoCommands.length; i++) {
      commandMap.put(autoCommands[i].toString(), autoCommands[i]);
      m_autoNames[i + 1] = autoCommands[i].toString();
    }

    m_selectCommand = new SelectCommand<>(commandMap, () -> m_chooser.getSelected());

    m_chooser = new SendableChooser<>();

    m_chooser.setDefaultOption("None", "None");
    for (String autoName : m_autoNames) {
      m_chooser.addOption(autoName, autoName);
    }

    SmartDashboard.putData("Auto chooser", m_chooser);
    SmartDashboard.putNumber((m_waitCommandKey), 0.0);

    m_autoCommand =
        new SequentialCommandGroup(
                new VariableWaitCommand(() -> SmartDashboard.getNumber(m_waitCommandKey, 0.0)),
                m_selectCommand)
            .finallyDo(
                () ->
                    Robot.swerve.setOperatorPerspectiveForward(
                        Robot.state.getAlliance() == Alliance.Red
                            ? Rotation2d.fromDegrees(180)
                            : Rotation2d.fromDegrees(0)));
  }

  public Command getSelectedAutoCommand() {
    return m_autoCommand;
  }
}
