package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveAtSpeed;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.util.PressToContinue;

public class ManualLineUpClimb extends SequentialCommandGroup {
  private static class Print extends Command {
    private String m_msg;

    public Print(String msg) {
      m_msg = msg;
    }

    @Override
    public void initialize() {
      System.out.println("[ManualLineUpClimb] " + m_msg);
    }

    public boolean isFinished() {
      return true;
    }
  }

  private static class AdjustNote extends Command {
    public AdjustNote() {
      addRequirements(Robot.endAffector);
    }

    @Override
    public void execute() {
      double axisSpeed = 0.0;

      if (Robot.codriverControls.getRightTriggerAxis() > 0) {
        axisSpeed = Robot.codriverControls.getRightTriggerAxis();
      }
      if (Robot.codriverControls.getLeftTriggerAxis() > 0) {
        axisSpeed = -Robot.codriverControls.getLeftTriggerAxis();
      }

      Robot.endAffector.setAxisSpeed(axisSpeed);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }

  public ManualLineUpClimb(Trigger continueButton) {
    super(
        new Print("Extending arm, line it up with the bottom of the stage."),
        new ParallelDeadlineGroup(
            new PressToContinue(continueButton),
            new ShooterStop(),
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_PREPOSE_SPEED, CLIMBER.PREPOSE_ROTATIONS),
            new ExtensionArmMoveToRotations(EXTENSION_ARM.STAGE_LINE_UP_ROTATIONS),
            new DefaultDrive()),
        new Print("Lining up hooks on chain"),
        new ParallelCommandGroup(
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_VERTICAL_SPEED, CLIMBER.VERTICAL_ROTATIONS),
            new ExtensionArmMoveToRotations(EXTENSION_ARM.HOME_ROTATIONS),
            new DriveAtSpeed(
                SWERVE.DISTANCE_FROM_STAGE_TO_CHAIN / SWERVE.SECONDS_FROM_STAGE_TO_CHAIN,
                0,
                SWERVE.SECONDS_FROM_STAGE_TO_CHAIN)),
        new Print("Ensure hooks are engaged. Adjust if needed"),
        new Print("Setting hooks"),
        new ClimberRotatePastRotations(CLIMBER.SET_HOOKS_SPEED, CLIMBER.SET_HOOKS_ROTATIONS),
        new Print("Ensure Hooks are set. Adjust if needed"),
        new Print("Positioning to transfer note"),
        new ParallelCommandGroup(
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_EXTENSION_SPEED, CLIMBER.PAST_EXTENSION_ROTATIONS),
            new DriveAtSpeed(
                -(SWERVE.DISTANCE_TO_ROTATE_PAST_EXTENSION
                    / SWERVE.SECONDS_TO_ROTATE_PAST_EXTENSION),
                0,
                SWERVE.SECONDS_TO_ROTATE_PAST_EXTENSION)),
        new Print("Check position before extending arm to climb"),
        new PressToContinue(continueButton),
        new Print("Extending arm to climb. Ready to climb!"),
        new ParallelCommandGroup(
            new DriveAtSpeed(
                SWERVE.DISTANCE_TO_READY_CLIMB / SWERVE.SECONDS_TO_READY_CLIMB,
                0,
                SWERVE.SECONDS_TO_READY_CLIMB)
            // new ClimberRotatePastRotations(, 0)
            ),
        new ExtensionArmMoveToRotations(EXTENSION_ARM.CLIMB_ONLY_ROTATIONS),
        new ClimberLevelClimb());
  }
}
