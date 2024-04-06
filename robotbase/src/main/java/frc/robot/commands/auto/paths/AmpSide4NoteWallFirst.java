package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.drive.TranslateToGamepiece;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.intake.Pickup;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class AmpSide4NoteWallFirst extends SequentialCommandGroup {
  public AmpSide4NoteWallFirst() {
    super(
        // Preload on the move (future Tyson problem) + Drive to note 2
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("AmpSide4Note1WallFirst.1", true),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        // drive to and pickup second note
        new DriveChoreoPath("AmpSide4Note1WallFirst.2", false),
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new Pickup()),

        // Drive back with note 2 and shoot
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("AmpSide4Note2WallFirst.1", false).deadlineWith(new Pickup()),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),

        // Drive to and pickup third note
        new DriveChoreoPath("AmpSide4Note2WallFirst.2", false),
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new Pickup()),

        // Drive back with note 3 and shoot
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("AmpSide4Note3WallFirst.1", false).deadlineWith(new Pickup()),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        new DriveChoreoPath("AmpSide4Note3WallFirst.2"),
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new Pickup()));
  }

  @Override
  public String toString() {
    return "Amp Side 4 Note Wall FIrst";
  }
}
