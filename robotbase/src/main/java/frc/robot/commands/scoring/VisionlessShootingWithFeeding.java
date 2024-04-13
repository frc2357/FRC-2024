package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SCORING;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class VisionlessShootingWithFeeding extends ParallelDeadlineGroup {
  public VisionlessShootingWithFeeding(double shooterRPM, double pivotAngle) {
    super(
        new SequentialCommandGroup(
            new ShooterWaitForRPM().withTimeout(SCORING.VISIONLESS_SHOT_WAIT_TO_FIRE_SECONDS),
            new IntakeFeedToShooter().withTimeout(0.75)),
        new ShooterSetRPM(shooterRPM),
        new PivotHoldAngle(pivotAngle, true, true));
  }
}
