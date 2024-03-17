package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.paths.Centerline2Speaker;
import frc.robot.commands.auto.paths.Close3AndRUN;
import frc.robot.commands.auto.paths.Close3Speaker;
import frc.robot.commands.auto.paths.LeftClose1Speaker;
import frc.robot.commands.auto.paths.MiddleClose1Speaker;
import frc.robot.commands.auto.paths.RightClose1Speaker;
import frc.robot.commands.auto.paths.ShootAndNothing;

public class AutoCommandChooser {
  private Command[] m_autoCommands;
  private SendableChooser<Command> m_chooser;

  private String m_waitCommandKey = "wait";

  public AutoCommandChooser() {

    m_autoCommands =
        new Command[] {
          // new DriveChoreoPath("Line"),
          new Close3Speaker(),
          new Centerline2Speaker(),
          new LeftClose1Speaker(),
          new MiddleClose1Speaker(),
          new RightClose1Speaker(),
          new ShootAndNothing(),
          new Close3AndRUN(),
        };

    m_chooser = new SendableChooser<>();

    m_chooser.setDefaultOption("None", new WaitCommand(0));
    for (Command autoCommand : m_autoCommands) {
      m_chooser.addOption(autoCommand.toString(), autoCommand);
    }
    SmartDashboard.putData("Auto chooser", m_chooser);
    SmartDashboard.putNumber((m_waitCommandKey), 0.0);
  }

  public Command getWaitCommand() {
    double waitTime = SmartDashboard.getNumber(m_waitCommandKey, 0.0);
    return new WaitCommand(waitTime);
  }

  public Command getSelectedAutoCommand() {
    return new SequentialCommandGroup(getWaitCommand(), m_chooser.getSelected());
  }
}
