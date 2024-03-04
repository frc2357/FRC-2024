package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LEDsSetColor extends Command {

  Color m_color;

  public LEDsSetColor(Color color) {
    addRequirements(Robot.LEDs);
    m_color = color;
  }

  @Override
  public void initialize() {
    Robot.LEDs.setColor(m_color);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
