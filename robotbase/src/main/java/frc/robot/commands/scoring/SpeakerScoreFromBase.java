package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.pivot.PivotSetRotation;
import frc.robot.commands.shooter.ShooterSetRPMs;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWaitForRPMs;

public class SpeakerScoreFromBase extends SequentialCommandGroup {
  public SpeakerScoreFromBase() {
    super(
        new ShooterSetRPMs(
            SHOOTER.TOP_MOTOR_SPEAKER_SCORE_FROM_BASE_RPMS,
            SHOOTER.BOTTOM_MOTOR_SPEAKER_SCORE_FROM_BASE_RPMS),
        new PivotSetRotation(PIVOT.SPEAKER_SCORE_FROM_BASE_ROTATION),
        new ShooterWaitForRPMs(),
        new IntakeFeedToShooter().withTimeout(INTAKE.FEED_TO_SHOOTER_TIMEOUT),
        new ShooterStop(),
        new IntakeStop()
        // new SetRobotStateCommand(RobotState.State.EMPTY)

        );
  }
}
