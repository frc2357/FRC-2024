package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.controls.util.RumbleInterface;

public class DriverControls implements RumbleInterface {
  private XboxController m_controller;
  private double m_deadband;

  private JoystickButton m_backButton;
  private JoystickButton m_startButton;

  public DriverControls(XboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_backButton = new JoystickButton(m_controller, Button.kBack.value);
    m_startButton = new JoystickButton(m_controller, Button.kStart.value);

    mapControls();
  }

  public void mapControls() {
    m_backButton.onTrue(new InstantCommand(() -> Robot.drive.setYaw(0)));
    m_startButton.onTrue(new InstantCommand(() -> Robot.drive.setYaw(180)));
  }

  public double getX() {
    return -modifyAxis(m_controller.getLeftX());
  }

  public double getY() {
    System.out.println(-modifyAxis(m_controller.getLeftY()));
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
