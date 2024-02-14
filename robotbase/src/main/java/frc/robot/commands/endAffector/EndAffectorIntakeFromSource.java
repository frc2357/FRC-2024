package frc.robot.commands.endAffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.Robot;

public class EndAffectorIntakeFromSource extends Command {

  public EndAffectorIntakeFromSource() {
    addRequirements(Robot.endAffector);
  }

  @Override
  public void initialize() {
    Robot.endAffector.setSpeed(END_AFFECTOR.INTAKE_SPEED);
  }

  @Override
  public boolean isFinished() {
    return Robot.endAffector.getMotorAmperage() >= END_AFFECTOR.STOWED_NOTE_AMPERAGE_LIMIT;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.endAffector.setSpeed(0);
  }
}
