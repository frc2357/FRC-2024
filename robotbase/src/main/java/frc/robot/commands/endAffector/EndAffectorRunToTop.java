package frc.robot.commands.endAffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Robot;

public class EndAffectorRunToTop extends Command {
  public EndAffectorRunToTop() {
    addRequirements(Robot.endAffector);
  }

  @Override
  public void initialize() {
    Robot.endAffector.setProximitySensorPower(true);
    Robot.endAffector.setSpeed(END_AFFECTOR.INTAKE_SPEED);
  }

  @Override
  public boolean isFinished() {
    // return Robot.endAffector.getProximitySensor();
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.endAffector.setSpeed(0);
    Robot.endAffector.setProximitySensorPower(false);
  }
}
