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
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.drive.TranslateToGamepiece;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class SourceSide4Note extends SequentialCommandGroup {
  public SourceSide4Note() {
    super(
        // Preload on the move (future Tyson problem) + Drive to note 2
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

        // Pickup second note
        new ParallelDeadlineGroup(new TranslateToGamepiece(3), new AutoPickup()),

        // Drive back with note 2 and shoot
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("SourceSide4Note2", 0, false).deadlineWith(new AutoPickup()),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        new ShooterStop(),

        // Drive to and pickup third note
        new DriveChoreoPath("SourceSide4Note2", 1, false),
        new ParallelDeadlineGroup(new TranslateToGamepiece(3, 0.9), new AutoPickup()),

        // Drive back with note 3 and shoot
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("SourceSide4Note3", 0, false).deadlineWith(new AutoPickup()),
                new TargetLockOnSpeaker(true).withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new ShooterWaitForRPM(),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        new ShooterStop(),
        new DriveChoreoPath("SourceSide4Note3", 1, false),
        new ParallelDeadlineGroup(new TranslateToGamepiece(3, 0.9), new AutoPickup()),

        // Drive back with note 4 and shoot
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("SourceSide4Note4", 0, false, true)
                    .deadlineWith(new AutoPickup()),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)));
  }

  @Override
  public String toString() {
    return "Source Side 4 Note";
  }
}
