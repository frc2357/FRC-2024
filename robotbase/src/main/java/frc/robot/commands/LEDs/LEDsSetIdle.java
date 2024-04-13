package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LEDsSetIdle extends Command {

  @Override
  public void end(boolean interrupted) {
    Robot.leds.setIdle();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
