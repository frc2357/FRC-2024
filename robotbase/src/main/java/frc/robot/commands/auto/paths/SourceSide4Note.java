package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.drive.TranslateToGamepiece;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.intake.IntakeNoteFromFloor;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class SourceSide4Note extends SequentialCommandGroup {
  public SourceSide4Note() {
    super(
        // Preload on the move (future Tyson problem) + Drive to note 2
        new ParallelDeadlineGroup(
            new DriveChoreoPath("SourceSide4Note1.1", true)
                .handleInterrupt(() -> System.out.println("Interrupted")),
            new SequentialCommandGroup(
                new WaitCommand(0.75), new IntakeFeedToShooter().withTimeout(0.25)),
            new PivotHoldAngle(38),
            new ShooterSetRPM(4000)),
        // new DriveChoreoPath("SourceSide4Note1.1", true),

        // Pickup second note
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new IntakeNoteFromFloor()),

        // Drive back with note 2 and shoot
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("SourceSide4Note2.1"),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4000)),

        // Drive to and pickup third note
        new DriveChoreoPath("SourceSide4Note2.2"),
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new IntakeNoteFromFloor()),

        // Drive back with note 3 and shoot
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("SourceSide4Note3.1"),
                new TargetLockOnSpeaker(true).withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new ShooterWaitForRPM(),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4000)),

        // Drive to and pickup fourth note
        new DriveChoreoPath("SourceSide4Note3.2"),
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new IntakeNoteFromFloor()),
        new DriveChoreoPath("SourceSide4Note4.1"));
    // Drive back with note 4 and shoot
    // new ParallelDeadlineGroup(
    // new SequentialCommandGroup(
    // new DriveChoreoPath("SourceSide4Note4.1"),
    // new
    // TargetLockOnSpeaker(true).withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
    // new ShooterWaitForRPM(),
    // new IntakeFeedToShooter().withTimeout(0.2)),
    // new VisionTargeting(4000)));
  }

  @Override
  public String toString() {
    return "SourceSide4Note";
  }
}
