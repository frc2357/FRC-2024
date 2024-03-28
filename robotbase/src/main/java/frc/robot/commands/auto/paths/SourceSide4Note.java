package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPM;

public class SourceSide4Note extends SequentialCommandGroup {
  public SourceSide4Note() {
    super(
        // Preload on the move (future Tyson problem) + Drive to note 2
        new ParallelDeadlineGroup(
            new DriveChoreoPath("SourceSide4Note1.1"),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new WaitCommand(1), new IntakeFeedToShooter().withTimeout(0.25)),
                new PivotHoldAngle(45),
                new ShooterSetRPM(3500)))

        // Pickup second note
        // new ParallelDeadlineGroup(
        //    new TranslateToGamepiece(),
        //    new IntakeNoteFromFloor()
        // ),

        // // Drive back with note 2 and shoot
        // new ParallelDeadlineGroup(
        //    new SequentialCommandGroup(
        //       new DriveChoreoPath("SourceSide4Note2.1"),
        //       new TargetLockOnSpeaker().withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
        //       new IntakeFeedToShooter().withTimeout(0.2)
        //    ),
        //    new VisionTargeting()
        // ),

        // // Drive to and pickup third note
        // new DriveChoreoPath("SourceSide4Note2.2"),
        // new ParallelDeadlineGroup(
        //    new TranslateToGamepiece(),
        //    new IntakeNoteFromFloor()
        // ),

        // // Drive back with note 3 and shoot
        // new ParallelDeadlineGroup(
        //    new SequentialCommandGroup(
        //       new DriveChoreoPath("SourceSide4Note3.1"),
        //       new TargetLockOnSpeaker().withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
        //       new IntakeFeedToShooter().withTimeout(0.2)
        //    ),
        //    new VisionTargeting()
        // ),

        // // Drive to and pickup fourth note
        // new DriveChoreoPath("SouceSide4Note3.2"),
        // new ParallelDeadlineGroup(
        //    new TranslateToGamepiece(),
        //    new IntakeNoteFromFloor()
        // ),

        // // Drive back with note 4 and shoot
        // new ParallelDeadlineGroup(
        //    new SequentialCommandGroup(
        //       new DriveChoreoPath("SourceSide4Note4.1"),
        //       new TargetLockOnSpeaker().withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
        //       new IntakeFeedToShooter().withTimeout(0.2)
        //    ),
        //    new VisionTargeting()
        // )
        );
  }

  @Override
  public String toString() {
    return "SourceSide4Note";
  }
}
