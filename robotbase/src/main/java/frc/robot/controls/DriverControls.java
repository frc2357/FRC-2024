package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.LEDs.LEDsSetIdle;
import frc.robot.commands.climber.ManualLineUpTrap;
import frc.robot.commands.drive.TargetLockForFeeding;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.intake.CancelIntakeOnEnd;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.pickup.VisionPickup;
import frc.robot.commands.scoring.AmpSequence;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.commands.scoring.VisionlessShooting;
import frc.robot.commands.scoring.VisionlessShootingWithFeeding;
import frc.robot.commands.shooter.ShooterWaitForRPM;
import frc.robot.commands.source.SourceIntakeFromShooter;
import frc.robot.controls.util.AxisInterface;
import frc.robot.controls.util.AxisThresholdTrigger;
import frc.robot.controls.util.RumbleInterface;

public class DriverControls implements RumbleInterface {
  private XboxController m_controller;
  private double m_deadband;

  public JoystickButton m_aButton;
  private JoystickButton m_bButton;
  private JoystickButton m_xButton;
  private JoystickButton m_yButton;

  private JoystickButton m_backButton;
  private JoystickButton m_startButton;
  public JoystickButton m_leftBumper;
  public JoystickButton m_rightBumper;

  private AxisThresholdTrigger m_rightTriggerPrime;
  private AxisThresholdTrigger m_rightTriggerShoot;
  private AxisThresholdTrigger m_leftTrigger;

  private POVButton m_upDPad;
  private POVButton m_rightDPad;
  private POVButton m_downDPad;
  private POVButton m_leftDPad;

  public DriverControls(XboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_aButton = new JoystickButton(m_controller, Button.kA.value);
    m_bButton = new JoystickButton(m_controller, Button.kB.value);
    m_xButton = new JoystickButton(m_controller, Button.kX.value);
    m_yButton = new JoystickButton(m_controller, Button.kY.value);

    m_backButton = new JoystickButton(m_controller, Button.kBack.value);
    m_startButton = new JoystickButton(m_controller, Button.kStart.value);

    m_leftBumper = new JoystickButton(controller, Button.kLeftBumper.value);
    m_rightBumper = new JoystickButton(controller, Button.kRightBumper.value);

    m_rightTriggerPrime = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger, 0.0);
    m_rightTriggerShoot = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger, .8);
    m_leftTrigger = new AxisThresholdTrigger(m_controller, Axis.kLeftTrigger, 0);

    m_upDPad = new POVButton(m_controller, 0);
    m_rightDPad = new POVButton(m_controller, 90);
    m_downDPad = new POVButton(m_controller, 180);
    m_leftDPad = new POVButton(m_controller, 270);

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

    Trigger noLeftBumper = m_leftBumper.negate();

    Trigger isClimbing =
        new Trigger(
            () -> {
              return Robot.state.isClimbing();
            });

    Trigger notClimbing = isClimbing.negate();

    m_aButton
        .and(notClimbing)
        .whileTrue(
            new VisionlessShootingWithFeeding(Robot.shooterCurve[1][2], Robot.shooterCurve[1][1]));
    m_bButton.whileTrue(
        new VisionlessShootingWithFeeding(Robot.shooterCurve[4][2], Robot.shooterCurve[4][1]));

    m_leftTrigger.and(notClimbing).onTrue(new VisionPickup());
    m_leftTrigger.and(notClimbing).toggleOnFalse(new CancelIntakeOnEnd());

    m_leftBumper.whileTrue(
        new ParallelCommandGroup(new TargetLockForFeeding(), new VisionlessShooting(3650, 38)));
    m_leftBumper
        .and(m_rightTriggerShoot)
        .and(notClimbing)
        .whileTrue(
            new SequentialCommandGroup(
                new ShooterWaitForRPM(), new IntakeFeedToShooter().withTimeout(0.25)));

    m_yButton.onTrue(
        new ManualLineUpTrap(m_yButton, m_aButton.and(isClimbing))
            .finallyDo(
                () -> {
                  Robot.state.setClimbing(false);
                }));

    m_xButton.whileTrue(new SourceIntakeFromShooter());

    // scoring
    m_rightBumper.onTrue(new AmpSequence(m_rightBumper));

    m_backButton.onTrue(new InstantCommand(() -> Robot.swerve.zeroGyro(false)));
    m_startButton.onTrue(new InstantCommand(() -> Robot.swerve.zeroGyro(true)));

    m_rightTriggerPrime
        .and(noLeftBumper)
        .and(notClimbing)
        .whileTrue(
            new ParallelCommandGroup(new VisionTargeting(4000, true), new TargetLockOnSpeaker()));
    m_rightTriggerShoot
        .and(noLeftBumper)
        .and(notClimbing)
        .whileTrue(
            new SequentialCommandGroup(
                new ShooterWaitForRPM().withTimeout(1.5),
                new IntakeFeedToShooter(),
                new LEDsSetIdle()));
  }

  public double getX() {
    return Robot.state.isAdjusting() ? 0 : -modifyAxis(m_controller.getLeftX());
  }

  public double getY() {
    return Robot.state.isAdjusting() ? 0 : -modifyAxis(m_controller.getLeftY());
  }

  public double getLeftTrigger() {
    return Robot.state.isClimbing() ? 0 : m_controller.getLeftTriggerAxis();
  }

  public boolean isLeftTriggerPressed() {
    return Robot.state.isClimbing() ? false : m_leftTrigger.getAsBoolean();
  }

  public double getRotation() {
    return Robot.state.isAdjusting() ? 0 : -modifyAxis(m_controller.getRightX());
  }

  /** Only for climb, don't use ever unless Nolan says so */
  public double getLeftStickY() {
    return m_controller.getLeftY();
  }

  public double getRightTriggerAxis() {
    return m_controller.getRightTriggerAxis();
  }

  public double getLeftTriggerAxis() {
    return m_controller.getLeftTriggerAxis();
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
