package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.shooter.ShooterWaitForRPM;
import frc.robot.commands.util.PressToContinue;

public class VisionShot extends SequentialCommandGroup {
  public VisionShot(Trigger fireTrigger) {
    super(
        new ParallelDeadlineGroup(
            new PressToContinue(fireTrigger), new VisionTargeting(), new TargetLockOnSpeaker()),
        new ShooterWaitForRPM(),
        new IntakeFeedToShooter());
  }
}
