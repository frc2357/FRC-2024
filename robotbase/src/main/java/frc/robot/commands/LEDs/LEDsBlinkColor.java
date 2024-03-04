package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LEDsBlinkColor extends Command {

  Timer timer = new Timer();
  Color m_color;
  double m_seconds;

  public LEDsBlinkColor(Color color, double seconds) {
    addRequirements(Robot.LEDs);
    m_color = color;
    m_seconds = seconds;
  }

  @Override
  public void initialize() {
    timer.start();
    Robot.LEDs.setColor(m_color);
  }

  @Override
  public void execute() {
    if (timer.advanceIfElapsed(m_seconds)) {
      Robot.LEDs.toggleColor(m_color);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    Robot.LEDs.stop();
  }
}
