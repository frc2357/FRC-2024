package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SCORING;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class SubwooferShot extends ParallelDeadlineGroup {
  public SubwooferShot() {
    super(
        new SequentialCommandGroup(
            new ShooterSetRPM(SCORING.SUBWOOFER_SHOT_SHOOTER_RPMS),
            new ShooterWaitForRPM().withTimeout(SCORING.SUBWOOFER_SHOT_WAIT_TO_FIRE_SECONDS),
            new IntakeFeedToShooter().withTimeout(0.25),
            new ShooterStop()),
        new PivotHoldAngle(SCORING.SUBWOOFER_SHOT_PIVOT_ANGLE));
  }
}
