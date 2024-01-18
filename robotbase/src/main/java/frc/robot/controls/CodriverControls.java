package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.controls.util.RumbleInterface;
import frc.robot.controls.util.XboxRaw;

public class CodriverControls implements RumbleInterface {

  private XboxController m_controller;
  private double m_deadband;

  public JoystickButton m_rightBumper;
  public JoystickButton m_leftBumper;

  public CodriverControls(XboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_rightBumper = new JoystickButton(m_controller, XboxRaw.BumperRight.value);
    m_leftBumper = new JoystickButton(m_controller, XboxRaw.BumperLeft.value);

    mapControls();
  }

  private void mapControls() {
    Trigger bothBumpers = m_rightBumper.and(m_leftBumper);

    m_rightBumper.onTrue(
        new InstantCommand(() -> Robot.state.onDriverAllianceSelect(Alliance.Red)));
    m_leftBumper.onTrue(new InstantCommand(() -> Robot.state.onDriverAllianceSelect(Alliance.Red)));
    bothBumpers.onTrue(new InstantCommand(() -> Robot.state.onDriverAllianceSelect(null)));
  }

  @Override
  public void setRumble(RumbleType type, double intensity) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setRumble'");
  }
}
