package frc.robot.commands.endAffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Robot;
import frc.robot.state.RobotState.NoteState;

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
    Robot.endAffector.stop();

    if (!interrupted) {
      Robot.state.setNoteState(NoteState.END_AFFECTOR_PRELOAD);
    }
  }
}
