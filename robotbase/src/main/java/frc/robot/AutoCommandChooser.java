package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveChoreoPath;

public class AutoCommandChooser {
  private Command[] m_autoCommands;
  private SendableChooser<Command> m_chooser;

  public AutoCommandChooser() {

    m_autoCommands =
        new Command[] {
          new DriveChoreoPath("Line"),
          new DriveChoreoPath("Close3Speaker"),
          // new DriveChoreoPath("RotationTest"),
          // new DriveChoreoPath("BoxTest"),
          // new DriveChoreoPath("TriangleTest"),
          // new DriveChoreoPath("RightClose1Speaker"),
          // new DriveChoreoPath("LoadAmp"),
          // new DriveChoreoPath("RefSideFar1"),
          // new DriveChoreoPath("RefSideFar3"),
        };

    m_chooser = new SendableChooser<>();

    m_chooser.setDefaultOption("None", new WaitCommand(0));
    for (Command autoCommand : m_autoCommands) {
      m_chooser.addOption(autoCommand.toString(), autoCommand);
    }
    SmartDashboard.putData("Auto chooser", m_chooser);
  }

  public Command getSelectedAutoCommand() {
    return m_chooser.getSelected();
  }
}
