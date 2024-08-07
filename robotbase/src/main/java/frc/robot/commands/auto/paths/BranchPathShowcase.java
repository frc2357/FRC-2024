package frc.robot.commands.auto.paths;

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

public class BranchPathShowcase extends SequentialCommandGroup {
    // these are constants so that we can easily tune this without having to look for the numbers in the mess this will inevitably be.
    private static final double m_secondNotePitchTolerance = 5;

  public BranchPathShowcase() {
    super(
        // Preload on the move (future Tyson problem) + Drive to note 2
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("AmpSide4Note1.1", true),
                new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                    .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                new IntakeFeedToShooter().withTimeout(0.2)),
            new VisionTargeting(4800)),
        // Here comes the decision tree. I am dearly sorry to those who hate indents, for this is sin.
        new PathNode( 
            new SequentialCommandGroup(
                new DriveChoreoPath("BPNote2:N1T"),
                new ParallelDeadlineGroup(new TranslateToGamepiece(3), new AutoPickup()),
                        // Drive back with note 2 and shoot
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                    new DriveChoreoPath("AmpSide4Note2WallFirst.1", false)
                        .deadlineWith(new AutoPickup()),
                    new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                        .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                    new IntakeFeedToShooter().withTimeout(0.2)), new VisionTargeting(4800)),
            new PathNode(null, null, null, "[BP1] Node 2"))
        ,
            new SequentialCommandGroup(
                new DriveChoreoPath("BPNote2:N1F"),
                new ParallelDeadlineGroup(new TranslateToGamepiece(3), new AutoPickup()),
                // Drive back with note 2 and shoot
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new DriveChoreoPath("AmpSide4Note2.1", false, true).deadlineWith(new AutoPickup()),
                        new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM())
                            .withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
                        new IntakeFeedToShooter().withTimeout(0.2)),
                    new VisionTargeting(4800)),
                new PathNode(
                    
                , 
                
                , 
                
                , "[BP1] Node 3")
        )
        ,     
        () -> (Math.abs(Robot.intakeCam.getNoteTargetPitch()) <= m_secondNotePitchTolerance), "[BP1] Node 1")
        );
  }

  @Override
  public String toString() {
    return "Amp Side 4 Note";
  }
}
