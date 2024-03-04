package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDS;

public class LEDs extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  boolean ledsOn;

  public LEDs() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(LEDS.STRIP_LENGTH);

    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setColor(Color color) {

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }

    m_led.setData(m_ledBuffer);
    ledsOn = true;
  }

  public void toggleColor(Color color) {
    if (areLEDsOn()) {
      stop();
    } else {
      setColor(color);
    }
  }

  public void stop() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, new Color());
    }

    m_led.setData(m_ledBuffer);
    ledsOn = false;
  }

  public boolean areLEDsOn() {
    return ledsOn;
  }
}
