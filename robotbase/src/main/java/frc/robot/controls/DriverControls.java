package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Robot;
import frc.robot.commands.intake.IntakeNoteFromFloor;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.pivot.PivotStop;
import frc.robot.commands.scoring.AmpPrepose;
import frc.robot.commands.scoring.AmpScore;
import frc.robot.commands.scoring.SpeakerShotFire;
import frc.robot.commands.scoring.SpeakerShotPrime;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.controls.util.AxisInterface;
import frc.robot.controls.util.AxisThresholdTrigger;
import frc.robot.controls.util.RumbleInterface;
import frc.robot.state.RobotState.NoteState;

public class DriverControls implements RumbleInterface {
  private XboxController m_controller;
  private double m_deadband;

  private JoystickButton m_aButton;
  private JoystickButton m_xButton;
  private JoystickButton m_yButton;

  private JoystickButton m_backButton;
  private JoystickButton m_startButton;
  public JoystickButton m_leftBumper;
  public JoystickButton m_rightBumper;

  private AxisThresholdTrigger m_rightTriggerPrime;
  private AxisThresholdTrigger m_rightTriggerShoot;
  private AxisThresholdTrigger m_leftTrigger;

  private POVButton m_rightDPad;

  public DriverControls(XboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_aButton = new JoystickButton(m_controller, Button.kA.value);
    m_xButton = new JoystickButton(m_controller, Button.kX.value);
    m_yButton = new JoystickButton(m_controller, Button.kY.value);

    m_backButton = new JoystickButton(m_controller, Button.kBack.value);
    m_startButton = new JoystickButton(m_controller, Button.kStart.value);

    m_leftBumper = new JoystickButton(controller, Button.kLeftBumper.value);
    m_rightBumper = new JoystickButton(controller, Button.kRightBumper.value);

    m_rightTriggerPrime = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger, 0.1);
    m_rightTriggerShoot = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger, .7);
    m_leftTrigger = new AxisThresholdTrigger(m_controller, Axis.kLeftTrigger, 0);

    m_rightDPad = new POVButton(m_controller, 90);

    mapControls();
  }

  public double getRightStickYAxis() {
    return m_controller.getRightY();
  }

  public void mapControls() {
    AxisInterface righStickYAxis =
        () -> {
          return getRightStickYAxis();
        };

    m_backButton.onTrue(new InstantCommand(() -> Robot.swerve.setYaw(0)));
    m_startButton.onTrue(new InstantCommand(() -> Robot.swerve.setYaw(180)));

    m_leftTrigger.whileTrue(new IntakeNoteFromFloor());

    // m_rightBumper.onTrue(new DriverAmpScore());
    m_rightBumper.onTrue(
        new ConditionalCommand(
            new AmpScore(),
            new AmpPrepose(),
            () -> Robot.state.isNote(NoteState.END_AFFECTOR_PRELOAD)));

    // m_rightTriggerPrime.whileTrue(
    // new SpeakerShotPrime(
    // SHOOTER.TOP_MOTOR_SUBWOOFER_SHOT_RPMS,
    // SHOOTER.BOTTOM_MOTOR_SUBWOOFER_SHOT_RPMS,
    // PIVOT.SUBWOOFER_SHOT_ANGLE));

    m_rightTriggerPrime.whileTrue(
        new SpeakerShotPrime(
            Robot.shooter.getShooterCurveRow()[2],
            Robot.shooter.getShooterCurveRow()[2],
            Robot.shooter.getShooterCurveRow()[1]));
    m_rightTriggerPrime.onFalse(
        new ParallelCommandGroup(new ShooterStop(), new PivotStop(), new IntakeStop()));
    m_rightTriggerShoot.whileTrue(new SpeakerShotFire());
  }

  public double getX() {
    return -modifyAxis(m_controller.getLeftX());
  }

  public double getY() {
    return -modifyAxis(m_controller.getLeftY());
  }

  public double getRotation() {
    return -modifyAxis(m_controller.getRightX());
  }

  public double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public double modifyAxis(double value) {
    value = deadband(value, m_deadband);
    // value = Math.copySign(Math.pow(value,
    // Constants.SWERVE.TRANSLATION_RAMP_EXPONENT), value);
    return value;
  }

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
