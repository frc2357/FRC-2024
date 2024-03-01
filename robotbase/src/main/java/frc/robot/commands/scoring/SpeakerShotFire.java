package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INTAKE;
import frc.robot.Robot;

public class SpeakerShotFire extends Command {
  public SpeakerShotFire() {
    addRequirements(Robot.intake);
  }

  @Override
  public void execute() {
    if (Robot.shooter.isAtTargetSpeed() && Robot.pivot.isPivotAtAngle()) {
      Robot.intake.set(INTAKE.FEED_SPEED_PERCENT_OUTPUT);
    }
  }

  @Override
  public boolean isFinished() {
    return false; // Manually finish
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
