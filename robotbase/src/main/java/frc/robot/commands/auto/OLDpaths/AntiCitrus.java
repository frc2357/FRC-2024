package frc.robot.commands.auto.OLDpaths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoPickup;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.drive.TranslateToGamepiece;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class AntiCitrus extends SequentialCommandGroup {

  public AntiCitrus() {
    super(
        // Preload on the move (future Tyson problem) + Drive to note 2
        new ParallelDeadlineGroup(
            new DriveChoreoPath("AntiCitrus1Blue", 0, true),
            new SequentialCommandGroup(
                new WaitCommand(0.75), new IntakeFeedToShooter().withTimeout(0.25)),
            new PivotHoldAngle(35.5),
            new ShooterSetRPM(4200)),
        new ShooterStop(),

        // Pickup second note
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new AutoPickup()),

        // Drive back with note 2 and shoot
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("AntiCitrus2", 0, false).deadlineWith(new AutoPickup()),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(1.5),
                new IntakeFeedToShooter().withTimeout(1)),
            new VisionTargeting(4800)),
        new ShooterStop(),
        new ShooterStop());
  }

  public String toString() {
    return "ANTI-CITRUS";
  }
}
