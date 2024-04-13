package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
import frc.robot.commands.drive.DriveToStage;
import frc.robot.commands.endAffector.EndAffectorSetSpeed;
import frc.robot.commands.endAffector.EndAffectorStop;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterStop;

public class AutoClimb extends SequentialCommandGroup {
  private static class Print extends Command {
    private String m_msg;

    public Print(String msg) {
      m_msg = msg;
    }

    @Override
    public void initialize() {
      System.out.println("[AutoClimb] " + m_msg);
    }

    public boolean isFinished() {
      return true;
    }
  }

  private static class PressToContinue extends Command {
    private JoystickButton m_button;
    private boolean m_wasReleased;

    public PressToContinue(JoystickButton button) {
      m_button = button;
    }

    @Override
    public void initialize() {
      m_wasReleased = false;
    }

    @Override
    public boolean isFinished() {
      if (m_button.getAsBoolean()) {
        if (m_wasReleased) {
          return true;
        }
      } else {
        m_wasReleased = true;
      }
      return false;
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

  public AutoClimb(JoystickButton continueButton) {
    super(
        new Print("[AutoClimb] Starting Auto Climb..."),
        new ParallelCommandGroup(
            new ShooterStop(),
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_PREPOSE_SPEED, CLIMBER.PREPOSE_ROTATIONS),
            new SequentialCommandGroup(
                new Print("[AutoClimb] Lining up"),
                new ParallelDeadlineGroup(
                    new PressToContinue(continueButton), new DriveToStage()))),
        new ParallelDeadlineGroup(
            new Print("[AutoClimb] Manual line up adjustment, if needed"),
            new ParallelDeadlineGroup(new PressToContinue(continueButton), new DefaultDrive())),
        new ParallelCommandGroup(
            new Print("[AutoClimb] Driving under chain"),
            new DriveAtSpeed(
                -(SWERVE.DISTANCE_TO_UNDER_CHAIN / SWERVE.SECONDS_TO_UNDER_CHAIN),
                0,
                SWERVE.SECONDS_TO_UNDER_CHAIN)),
        new Print("[AutoClimb] Engaging hooks on chain"),
        new ParallelCommandGroup(
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_VERTICAL_SPEED, CLIMBER.VERTICAL_ROTATIONS),
            new DriveAtSpeed(
                SWERVE.DISTANCE_TO_TOUCH_CHAIN / SWERVE.SECONDS_TO_TOUCH_CHAIN,
                0,
                SWERVE.SECONDS_TO_TOUCH_CHAIN)),
        new Print("[AutoClimb] Ensure Hooks are engaged. Adjust if needed"),

        // Allow drive base to move now
        // TODO: Slow down drive for this
        new ParallelDeadlineGroup(new PressToContinue(continueButton), new DefaultDrive()),
        new Print("[AutoClimb] Setting hooks"),
        new ClimberRotatePastRotations(CLIMBER.SET_HOOKS_SPEED, CLIMBER.SET_HOOKS_ROTATIONS),
        new Print("[AutoClimb] Ensure Hooks are set. Adjust if needed"),

        // Allow drive base to move now
        // TODO: Slow down drive for this
        new ParallelDeadlineGroup(new PressToContinue(continueButton), new DefaultDrive()),
        new Print("[AutoClimb] Positioning to transfer note"),
        new ParallelCommandGroup(
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_EXTENSION_SPEED, CLIMBER.PAST_EXTENSION_ROTATIONS),
            new DriveAtSpeed(
                -(SWERVE.DISTANCE_TO_ROTATE_PAST_EXTENSION
                    / SWERVE.SECONDS_TO_ROTATE_PAST_EXTENSION),
                0,
                SWERVE.SECONDS_TO_ROTATE_PAST_EXTENSION)),
        new Print("[AutoClimb] Check position before transferring note"),
        new PressToContinue(continueButton),
        new Print("[AutoClimb] Transferring note"),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                // Note Preload
                new ExtensionArmMoveToRotations(EXTENSION_ARM.NOTE_STOW_ROTATIONS),

                // Run end affector, shooter, and intake to load note
                new ParallelDeadlineGroup(
                    new WaitCommand(SCORING.SECONDS_PRELOAD_NOTE),
                    new SequentialCommandGroup(new WaitCommand(0.25), new IntakeFeedToShooter()),
                    new EndAffectorSetSpeed(END_AFFECTOR.PRELOAD_SPEED),
                    new ShooterSetRPM(SHOOTER.FEED_END_AFFECTOR_RPM)),

                // Stop motors
                new ParallelCommandGroup(new IntakeStop(), new EndAffectorStop()),

                // Arm Prepose
                new ExtensionArmMoveToRotations(EXTENSION_ARM.TRAP_PREPOSE_ROTATIONS),
                new ParallelDeadlineGroup(
                    new WaitCommand(SCORING.SECONDS_PRELOAD_NOTE_FOR_TRAP),
                    new EndAffectorSetSpeed(END_AFFECTOR.PRELOAD_SPEED)),
                new EndAffectorStop()),

            // Hold until end of above command
            new PivotHoldAngle(PIVOT.END_AFFECTOR_PRELOAD_ANGLE)),
        new Print(
            "[AutoClimb] Co-driver adjust note now: right trigger is up, left trigger is down"),
        new ParallelDeadlineGroup(new PressToContinue(continueButton), new AdjustNote()),
        new ParallelCommandGroup(
            new ClimberRotatePastRotations(
                CLIMBER.ROTATE_PAST_READY_SPEED, CLIMBER.PAST_READY_ROTATIONS),
            new DriveAtSpeed(
                SWERVE.DISTANCE_TO_READY_TRAP / SWERVE.SECONDS_TO_READY_TRAP,
                0,
                SWERVE.SECONDS_TO_READY_TRAP),
            new ExtensionArmMoveToRotations(EXTENSION_ARM.TRAP_CLIMB_ROTATIONS)),
        new Print(
            "[AutoClimb] Ready to climb! Co-driver climbs using right trigger, press Y to score when in position"),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new PressToContinue(continueButton),
                new Print("[AutoClimb] Scoring note!"),
                new EndAffectorSetSpeed(END_AFFECTOR.SCORE_SPEED_TRAP),
                new PressToContinue(continueButton),
                new Print("[AutoClimb] Scored! Co-driver can keep the robot up for climbing"),
                new EndAffectorStop()),
            new ClimberLevelClimb()),
        new SequentialCommandGroup(
            new ClimberSpeed(-0.25, -0.25).withTimeout(1),
            new ExtensionArmMoveToRotations(EXTENSION_ARM.POST_TRAP_SCORE_ROTATIONS),
            new Print("[AutoClimb] Retracted extension arm."),
            new ClimberLevelClimb()));
  }
}
