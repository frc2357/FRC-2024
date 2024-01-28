package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.state.RobotState.DriveControlState;

public class EnableSpeakerLockCommand extends Command {
  @Override
  public void initialize() {
    Robot.state.setDriveControlState(DriveControlState.SPEAKER_LOCK);
    Robot.shooterLimelight.setPipeline(4);
  }

  // @Override
  // public boolean isFinished() {
  // return Utility.isWithinTolerance(Robot.shooterLimelight.getTX(), 0, 1);

  // }

  @Override
  public void end(boolean interrupted) {
    Robot.state.setDriveControlState(DriveControlState.FIELD_RELATIVE);
    Robot.shooterLimelight.setHumanPipelineActive();
  }
}
