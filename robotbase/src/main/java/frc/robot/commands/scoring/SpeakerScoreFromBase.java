package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PIVOT;
import frc.robot.Constants.SHOOTER;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pivot.PivotSetRotation;
import frc.robot.commands.shooter.ShooterSetRPMs;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWaitForRPMs;
import frc.robot.commands.state.SetRobotStateCommand;
import frc.robot.state.RobotState;

public class SpeakerScoreFromBase extends ParallelCommandGroup {
  public SpeakerScoreFromBase() {
    super(
        new ShooterSetRPMs(
            SHOOTER.TOP_MOTOR_SUBWOOFER_SHOT_RPMS, SHOOTER.BOTTOM_MOTOR_SUBWOOFER_SHOT_RPMS),
        new SequentialCommandGroup(
            new PivotSetRotation(PIVOT.SUBWOOFER_SHOT_ROTATION),
            new ShooterWaitForRPMs(),
            new SetRobotStateCommand(RobotState.State.EMPTY),
            new IntakeFeedToShooter(),
            new ShooterStop()));
  }
}
