package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class CancelIntakeOnEnd extends Command {

  private Command m_reserveIntake = new InstantCommand(() -> {}, Robot.intake);

  @Override
  public boolean isFinished() {
    return false;
  }

  public void end(boolean interrupted) {
    m_reserveIntake.schedule();
    Robot.leds.setIdle();
  }
}
