package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDS;

public class LEDs extends SubsystemBase {

  private AddressableLED m_LED;
  private AddressableLEDBuffer m_LEDBuffer;

  private Color m_lastColor = null;

  public static final Color MELTDOWN_ORANGE = new Color(50, 255, 0);
  public static final Color GREEN = new Color(255, 0, 0);
  public static final Color BLUE = new Color(0, 0, 255);
  public static final Color RED = new Color(0, 255, 0);

  public LEDs() {
    m_LED = new AddressableLED(LEDS.PORT_NUMBER);
    m_LEDBuffer = new AddressableLEDBuffer(LEDS.STRIP_LENGTH);

    m_LED.setLength(m_LEDBuffer.getLength());

    m_LED.setData(m_LEDBuffer);
    m_LED.start();
  }

  private void setColor(Color color) {
    if (color == m_lastColor) {
      return;
    }

    for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setLED(i, color);
    }

    m_LED.setData(m_LEDBuffer);
    m_lastColor = color;
  }

  public void setIntaking() {
    setColor(Color.kWhite);
  }

  public void setHasNote() {
    setColor(GREEN);
  }

  public void setIdle() {
    setColor(MELTDOWN_ORANGE);
  }

  public void setHasSpeakerTarget() {
    setColor(BLUE);
  }

  public void setNoSpeakerTarget() {
    setColor(RED);
  }
}
