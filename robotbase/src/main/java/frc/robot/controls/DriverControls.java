package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.commands.intake.IntakeNoteFromFloor;
import frc.robot.controls.util.AxisThresholdTrigger;
import frc.robot.controls.util.RumbleInterface;

public class DriverControls implements RumbleInterface {
  private XboxController m_controller;
  private double m_deadband;

  private JoystickButton m_backButton;
  private JoystickButton m_startButton;

  private AxisThresholdTrigger m_rightTriggerPrime;
  private AxisThresholdTrigger m_rightTriggerShoot;
  private AxisThresholdTrigger m_leftTrigger;

  public DriverControls(XboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_backButton = new JoystickButton(m_controller, Button.kBack.value);
    m_startButton = new JoystickButton(m_controller, Button.kStart.value);

    m_rightTriggerPrime = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger, .1);
    m_rightTriggerShoot = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger, .6);
    m_leftTrigger = new AxisThresholdTrigger(m_controller, Axis.kLeftTrigger, 0);

    mapControls();
  }

  public void mapControls() {
    m_backButton.onTrue(new InstantCommand(() -> Robot.swerve.setYaw(0)));
    m_startButton.onTrue(new InstantCommand(() -> Robot.swerve.setYaw(180)));

    // m_rightTriggerPrime.whileTrue(
    // new DriveToApriltag(
    // Constants.SWERVE.AMP_TY_SETPOINT,
    // Constants.SWERVE.AMP_ROTATION_SETPOINT,
    // Constants.SHOOTER_LIMELIGHT.AMP_PIPELINE_INDEX));

    m_leftTrigger.whileTrue(new IntakeNoteFromFloor());
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
  public void setRumble(RumbleType type, double intensity) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setRumble'");
  }
}
