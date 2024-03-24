package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Robot;

public class TurnOnProximitySensor extends Command {
  private Timer m_timer;

  public TurnOnProximitySensor() {
    m_timer = new Timer();
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    Robot.endAffector.setProximitySensorPower(true);
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(END_AFFECTOR.WAIT_FOR_PROXIMITY_SENSOR_SECONDS);
  }
}
