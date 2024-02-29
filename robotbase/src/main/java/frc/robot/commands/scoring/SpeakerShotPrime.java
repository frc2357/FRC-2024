package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SpeakerShotPrime extends Command {
  public SpeakerShotPrime() {
    addRequirements(Robot.shooter, Robot.pivot);
  }

  @Override
  public void initialize() {
    Robot.pivot.setAngle(Robot.shooter.getShooterCurveRow()[1]);
    Robot.shooter.setRPM(Robot.shooter.getShooterCurveRow()[2]);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.pivot.stop();
    Robot.shooter.stop();
  }
}
