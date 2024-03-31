package frc.robot.commands.auto.paths;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.drive.TranslateToGamepiece;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.intake.IntakeNoteFromFloor;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class SourceSide4NotePathplanner extends SequentialCommandGroup {
  public SourceSide4NotePathplanner() {
    super(
        // Preload on the move (future Tyson problem) + Drive to note 2
        new ParallelDeadlineGroup(
            new PathPlannerAuto("SourceSide4Note1Auto"),
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
                new PathPlannerAuto("SourceSide4Note2Auto"),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4000)),

        // Drive to and pickup third note
        new PathPlannerAuto("SourceSide4Note3Auto"),
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new IntakeNoteFromFloor()),

        // Drive back with note 3 and shoot
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new PathPlannerAuto("SourceSide4Note4Auto"),
                new TargetLockOnSpeaker(true).withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new ShooterWaitForRPM(),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4000)),

        // Drive to and pickup fourth note
        new PathPlannerAuto("SourceSide4Note5Auto"),
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new IntakeNoteFromFloor()),
        new PathPlannerAuto("SourceSide4Note6Auto"));
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
    return "SourceSide4NotePathplanner";
  }
}
