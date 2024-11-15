package frc.robot.commands.auto.OLDpaths;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoPickup;
import frc.robot.commands.auto.PathNode;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.drive.TranslateToGamepiece;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWaitForRPM;

// a real branched path but coded less dumb.
// simply go to where you see each note.
// if it is there, grab and score. if it is not, go the next as if you scored it, but directly
// travel.
// continue this pattern until you reach ALL the notes.
public class SourceSideBranchingPath extends SequentialCommandGroup {
  private double noteYawToleranceDegrees = 28;
  private double notePitchMaximumDegrees = 15;

  public SourceSideBranchingPath() {
    addCommands(
        // Preload on the move + Drive to N5
        new ParallelDeadlineGroup(
            new ConditionalCommand(
                new DriveChoreoPath("SourceSide4Note1Blue", 0, true),
                new DriveChoreoPath("SourceSide4Note1Red", 0, true),
                () -> Robot.state.getAlliance() == Alliance.Blue),
            new SequentialCommandGroup(
                new WaitCommand(0.75), new IntakeFeedToShooter().withTimeout(0.25)),
            new PivotHoldAngle(35.5),
            new ShooterSetRPM(4200)),
        new ShooterStop(),
        // checking N5
        new PathNode(
            grabN5ScoreAndPosition(), // if note there, grab and score
            new DriveChoreoPath("N5ToN4.1"), // if not, go to next.
            () -> isNoteInfrontOfBot(noteYawToleranceDegrees, notePitchMaximumDegrees),//CHECKS N5
            "[BP] Node for N5"),
        new PathNode(
            grabN4ScoreAndPosition(), // if note there, grab and score
            new DriveChoreoPath("N4ToN3.1"), // if not, go to next.
            () -> isNoteInfrontOfBot(noteYawToleranceDegrees, notePitchMaximumDegrees),//CHECKS N4
            "[BP] Node for N4"),
        new PathNode(
            grabN3ScoreAndPosition(), // if note there, grab and score
            new DriveChoreoPath("N3ToN2.1"), // if not, go to next.
            () -> isNoteInfrontOfBot(noteYawToleranceDegrees, notePitchMaximumDegrees),//CHECKS N3
            "[BP] Node for N3"),
        new PathNode(
            grabN2ScoreAndPosition(), // if note there, grab and score
            new DriveChoreoPath("N2ToTeleopStartPosition.1"), // if not, go to next.
            () -> isNoteInfrontOfBot(noteYawToleranceDegrees, notePitchMaximumDegrees),//CHECKS N2
            "[BP] Node for N2"));
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

  private SequentialCommandGroup grabN5ScoreAndPosition() {
    return new SequentialCommandGroup(
        // pick up N4
        new ParallelDeadlineGroup(new TranslateToGamepiece(3, 1), new AutoPickup()),
        // drive back and score N4
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("SourceSide4Note2", 0),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        // position for next note
        new DriveChoreoPath("SourceSide4Note2.2"));
  }



  private SequentialCommandGroup grabN4ScoreAndPosition() {
    return new SequentialCommandGroup(
        // pick up N3
        new ParallelDeadlineGroup(new TranslateToGamepiece(3, 1), new AutoPickup()),
        // drive back and score N3
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("SourceSide4Note3.1"),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        // position for N2
        new DriveChoreoPath("SourceSide4Note3.2"));
  }

  private SequentialCommandGroup grabN3ScoreAndPosition() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(new TranslateToGamepiece(3, 1), new AutoPickup()),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                // drive back with N2 and shoot
                new DriveChoreoPath("SourceSide4Note4.1")
                    .deadlineWith(new AutoPickup()),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        // position for teleop start
        new DriveChoreoPath("SourceSide4Note4.2"));
  }

  private SequentialCommandGroup grabN2ScoreAndPosition() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(new TranslateToGamepiece(3, 1.1), new AutoPickup()),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                // drive back with N2 and shoot
                new DriveChoreoPath("AmpSide4Note3WallFirst", 0, false)
                    .deadlineWith(new AutoPickup()),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        // position for N3
        new DriveChoreoPath("AmpSide4Note3WallFirst", 1));
  }

  @Override
  public String toString() {
    return "Source Side Branching Path";
  }
}
