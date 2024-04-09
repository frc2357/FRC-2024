package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LEDsSetHasNote extends Command {


  @Override
  public void end(boolean interrupted) {
    Robot.leds.setHasNote();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
