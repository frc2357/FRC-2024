package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SHOOTER;
import frc.robot.Robot;
import frc.robot.state.RobotState.NoteState;

public class DefaultShooter extends Command {
  public DefaultShooter() {
    addRequirements(Robot.shooter);
  }

  @Override
  public void execute() {
    if (Robot.state.isNote(NoteState.NOTE_STOWED)) {
      // Robot.shooter.setRPM(SHOOTER.DEFAULT_STOWED_RPMS);
    } else {
      Robot.shooter.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
