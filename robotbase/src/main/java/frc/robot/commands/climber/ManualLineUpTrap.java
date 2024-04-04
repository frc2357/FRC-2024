package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.Constants.PIVOT;
import frc.robot.Constants.SCORING;
import frc.robot.Constants.SHOOTER;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveAtSpeed;
import frc.robot.commands.endAffector.EndAffectorPreloadNote;
import frc.robot.commands.endAffector.EndAffectorSetSpeed;
import frc.robot.commands.endAffector.EndAffectorStop;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.state.SetNoteState;
import frc.robot.commands.util.PressToContinue;
import frc.robot.state.RobotState.NoteState;
import frc.robot.util.TurnOnProximitySensor;

public class ManualLineUpTrap extends SequentialCommandGroup {
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

  public ManualLineUpTrap(Trigger continueButton, Trigger scoreButton) {
    super(
        new Print("Turning on proximity sensor. Try not to blind anybody."),
        new TurnOnProximitySensor(),
        new Print("Extending Arm, line it up with the bottom of the stage."),
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
        new Print("Ensure Hooks are engaged. Adjust if needed"),

        // Allow drive base to move now
        // TODO: Slow down drive for this
        new Print("Setting hooks"),
        new ClimberRotatePastRotations(CLIMBER.SET_HOOKS_SPEED, CLIMBER.SET_HOOKS_ROTATIONS),
        new Print("Ensure Hooks are set. Adjust if needed"),

        // Allow drive base to move now
        // TODO: Slow down drive for this
        new Print("Positioning to transfer note"),
        new ParallelCommandGroup(
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_EXTENSION_SPEED, CLIMBER.PAST_EXTENSION_ROTATIONS)),
        // new DriveAtSpeed(
        // -(SWERVE.DISTANCE_TO_ROTATE_PAST_EXTENSION
        // / SWERVE.SECONDS_TO_ROTATE_PAST_EXTENSION),
        // 0,
        // SWERVE.SECONDS_TO_ROTATE_PAST_EXTENSION)),
        new Print("Check position before transferring note"),
        new PressToContinue(continueButton),
        new Print("Transferring note"),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                // Note Preload
                new ParallelCommandGroup(
                    new TurnOnProximitySensor(),
                    new ExtensionArmMoveToRotations(EXTENSION_ARM.NOTE_STOW_ROTATIONS)),

                new ParallelDeadlineGroup(
                  new EndAffectorPreloadNote(),
                  new IntakeFeedToShooter().beforeStarting(new WaitCommand(0.2)),
                  new ShooterSetRPM(SHOOTER.FEED_END_AFFECTOR_RPM),
                  new PivotHoldAngle(PIVOT.END_AFFECTOR_PRELOAD_ANGLE)
                ),

                // Arm Prepose
                new ExtensionArmMoveToRotations(EXTENSION_ARM.TRAP_PREPOSE_ROTATIONS),
                new SetNoteState(NoteState.END_AFFECTOR_PRELOAD))),

            // Hold until end of above command
        new Print("Co-driver adjust note: right trigger is up, left trigger is down"),
        new PressToContinue(continueButton),
        new ParallelCommandGroup(
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_READY_SPEED, CLIMBER.PAST_READY_ROTATIONS),
            new DriveAtSpeed(
                SWERVE.DISTANCE_TO_READY_TRAP / SWERVE.SECONDS_TO_READY_TRAP,
                0,
                SWERVE.SECONDS_TO_READY_TRAP),
            new ExtensionArmMoveToRotations(EXTENSION_ARM.TRAP_CLIMB_ROTATIONS)),
        // new ParallelDeadlineGroup(new PressToContinue(continueButton), new
        // AdjustNote()),
        new Print("Ready to climb! Co-driver using right trigger, press Y when in position"),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new PressToContinue(scoreButton),
                new Print("Scoring note!"),
                new EndAffectorSetSpeed(END_AFFECTOR.SCORE_SPEED_TRAP),
                new PressToContinue(continueButton),
                new EndAffectorStop()),
            new ClimberLevelClimb()),
        new SequentialCommandGroup(
            new ClimberSpeed(-0.25, -0.25).withTimeout(1),
            new ExtensionArmMoveToRotations(EXTENSION_ARM.POST_TRAP_SCORE_ROTATIONS),
            new Print("Retracted extension arm.")),
        new Print("Continue to adjust climb as needed."),
        new ClimberLevelClimb());
  }
}
