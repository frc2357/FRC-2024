package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.controls.util.RumbleInterface;

public class CodriverControls implements RumbleInterface {

  private XboxController m_controller;
  private double m_deadband;

  public CodriverControls(XboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    mapControls();
  }

  private void mapControls() {}

  @Override
  public void setRumble(RumbleType type, double intensity) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setRumble'");
  }
}
