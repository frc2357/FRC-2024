package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.climber.ClimberRotatePastRotations;
import frc.robot.commands.drive.DriveAtSpeed;
import frc.robot.commands.drive.DriveDistance;
import frc.robot.commands.drive.DriveToApriltag;
import frc.robot.commands.intake.IntakeNoteFromFloor;
import frc.robot.commands.scoring.AmpPrepose;
import frc.robot.commands.scoring.AmpScore;
import frc.robot.controls.util.AxisInterface;
import frc.robot.controls.util.AxisThresholdTrigger;
import frc.robot.controls.util.RumbleInterface;
import frc.robot.state.RobotState.State;

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

    m_rightTriggerPrime = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger, .1);
    m_rightTriggerShoot = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger, .6);
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
    m_aButton.onTrue(
        new ConditionalCommand(
            new AmpScore(), new AmpPrepose(), () -> Robot.state.isInState(State.AMP_PRE_POSE)));

    m_rightTriggerShoot.whileTrue(new DriveDistance(0, 1, 1));
    m_leftTrigger.whileTrue(new DriveAtSpeed(0, 1, 3));
    m_rightBumper.whileTrue(
        new DriveToApriltag(
            Constants.SWERVE.CLIMB_TY_SETPOINT,
            Robot.climber::getLineupOnStageRotationSetpoint,
            Constants.SHOOTER_PHOTON_CAMERA.APRIL_TAG_PIPELINE,
            Robot.shooterCam,
            false));
    m_leftBumper.whileTrue(new ClimberRotatePastRotations(0.2, 100));
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
