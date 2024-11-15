package frc.robot.commands.auto.OLDpaths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoPickup;
import frc.robot.commands.auto.PathNode;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.drive.TranslateToGamepiece;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.commands.shooter.ShooterWaitForRPM;

// a real branched path but coded less dumb.
// simply go to where you see each note.
// if it is there, grab and score. if it is not, go the next as if you scored it, but directly
// travel.
// continue this pattern until you reach ALL the notes.
public class AmpSideBranchingPath extends SequentialCommandGroup {
  private double noteYawToleranceDegrees = 28;
  private double notePitchMaximumDegrees = 3;

  public AmpSideBranchingPath() {
    addCommands(
        // Preload on the move + Drive to note 2
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("AmpSide4Note1WallFirst",0, true),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        // checking N2
        new DriveChoreoPath("AmpSide4Note1WallFirst", 1, false),
        new PathNode(
            grabN1ScoreAndPosition(), // if note there, grab and score
            new DriveChoreoPath("N1ToN2.1"), // if not, go to next.
            () -> isNoteInfrontOfBot(noteYawToleranceDegrees, notePitchMaximumDegrees),
            "[BP] Node for N1"),
        new PathNode(
            grabN2ScoreAndPosition(), // if note there, grab and score
            new DriveChoreoPath("N2ToN3.1"), // if not, go to next.
            () -> isNoteInfrontOfBot(noteYawToleranceDegrees, notePitchMaximumDegrees),
            "[BP] Node for N2"),
        new PathNode(
            grabN3ScoreAndPosition(), // if note there, grab and score
            new DriveChoreoPath("N3ToN4.1"), // if not, go to next.
            () -> isNoteInfrontOfBot(noteYawToleranceDegrees, notePitchMaximumDegrees),
            "[BP] Node for N3"),
        new PathNode(
            grabN4ScoreAndPosition(), // if note there, grab and score
            new DriveChoreoPath("N4ToTeleopStartPosition.1"), // if not, go to next.
            () -> isNoteInfrontOfBot(noteYawToleranceDegrees, notePitchMaximumDegrees),
            "[BP] Node for N4"));
  }

  private boolean isNoteInfrontOfBot(double yawToleranceDegrees, double maximumPitchDegrees) {
    System.out.println(
        "[BP] CURRENT CAMERA YAW: "
            + Robot.intakeCam.getNoteTargetYaw()
            + " | YAW TOLERANCE: "
            + yawToleranceDegrees
            + "\n[BP] CURRENT CAMERA PITCH: "
            + Robot.intakeCam.getNoteTargetPitch()
            + " | MAX ALLOWED PITCH: "
            + maximumPitchDegrees);

    return (Math.abs(Robot.intakeCam.getNoteTargetYaw()) <= yawToleranceDegrees)
        && Robot.intakeCam.getNoteTargetPitch() <= maximumPitchDegrees;
  }

  private SequentialCommandGroup grabN1ScoreAndPosition() {
    return new SequentialCommandGroup(
        // pick up N1
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new AutoPickup()),
        // drive back with N1 and shoot
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("AmpSide4Note2WallFirst")
                    .deadlineWith(new AutoPickup()),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),

        // position for N2
        new DriveChoreoPath("AmpSide4Note2WallFirst", 1));
  }

  private SequentialCommandGroup grabN2ScoreAndPosition() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(new TranslateToGamepiece(3, 1.1), new AutoPickup()),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                // drive back with N2 and shoot
                new DriveChoreoPath("AmpSide4Note3WallFirst")
                    .deadlineWith(new AutoPickup()),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        // position for N3
        new DriveChoreoPath("AmpSide4Note3WallFirst", 1));
  }

  private SequentialCommandGroup grabN3ScoreAndPosition() {
    return new SequentialCommandGroup(
        // pick up N3
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new AutoPickup()),
        // drive back and score N3
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("AmpSide4Note4.1"),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        // position for N4
        new DriveChoreoPath("AmpSide4Note4", 1));
  }

  private SequentialCommandGroup grabN4ScoreAndPosition() {
    return new SequentialCommandGroup(
        // pick up N3
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new AutoPickup()),
        // drive back and score N3
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("BPScoreN4", 0),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        // position for teleop start
        new DriveChoreoPath("BPScoreN4", 1));
  }

  @Override
  public String toString() {
    return "Amp Side Branching Path";
  }
}
