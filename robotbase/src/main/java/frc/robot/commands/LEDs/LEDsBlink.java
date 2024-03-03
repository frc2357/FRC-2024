package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LEDsBlink extends Command {

  Timer timer = new Timer();
  Color m_color;
  double seconds;

  public LEDsBlink(Color color, double hz) {
    addRequirements(Robot.leds);
    m_color = color;
    seconds = 1 / hz;
  }

  @Override
  public void initialize() {
    timer.start();
    Robot.leds.setColor(m_color);
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(seconds)) {
      Robot.leds.toggle(m_color);
      timer.restart();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    Robot.leds.stop();
  }
}
