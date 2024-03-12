package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.scoring.VisionTargeting;

public class AutoShoot extends ParallelDeadlineGroup {
  public AutoShoot() {
    super(
        new SequentialCommandGroup(
            new AutoWaitForTargeting().withTimeout(1), new IntakeRun(0.75, true).withTimeout(0.5)),
        new VisionTargeting(),
        new AutoTargetLockOnSpeaker());
  }
}
