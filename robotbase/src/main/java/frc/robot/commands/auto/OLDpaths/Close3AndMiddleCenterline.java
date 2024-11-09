package frc.robot.commands.auto.OLDpaths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.auto.AutoPickup;
import frc.robot.commands.auto.AutoPivotSetAngle;
import frc.robot.commands.auto.AutoShooterSetRPMAndFinish;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.drive.TranslateToGamepiece;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class Close3AndMiddleCenterline extends SequentialCommandGroup {
  public Close3AndMiddleCenterline() {
    super(
        // Preload Shot
        new AutoPivotSetAngle(60),
        new AutoShooterSetRPMAndFinish(3000),
        new ShooterWaitForRPM().withTimeout(0.5),
        new IntakeRun(0.75, false).withTimeout(0.5),

        // Shoot first note on the move and drive to centerline
        new AutoShooterSetRPMAndFinish(4000).alongWith(new AutoPivotSetAngle(33)),
        new IntakeRun(0.75, false).withTimeout(0.1),
        new DriveChoreoPath("CenterSub5Note1", 0, true, false),
        // Grab the note from the centerline
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new AutoPickup()),

        // Drive back with note 2 and shoot
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("CenterSub5Note2", 0, false, false),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.4)),
            new VisionTargeting(4250)),

        // Grab note 3 and shoot it
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("CenterSub5Note2", 1, false, false)
                    .deadlineWith(new AutoPickup()),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.4)),
            new VisionTargeting(4450)),

        // Grab note 4 and shoot it
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("CenterSub5Note2", 2, false, false)
                    .deadlineWith(new AutoPickup()),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.4)),
            new VisionTargeting(4250)));
  }

  @Override
  public String toString() {
    return "Close 3 And Middle Centerline";
  }
}
