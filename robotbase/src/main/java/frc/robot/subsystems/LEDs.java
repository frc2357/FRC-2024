package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDS;

public class LEDs extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  boolean rightLEDsOn = false;
  boolean leftLEDsOn = false;

  public LEDs() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(LEDS.STRIP_LEFT_LENGTH + LEDS.STRIP_RIGHT_LENGTH);

    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setColor(Color color) {

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }

    m_led.setData(m_ledBuffer);
    rightLEDsOn = true;
    leftLEDsOn = true;
  }

  public void setColorLeft(Color color) {
    for (int i = 0; i < LEDS.STRIP_LEFT_LENGTH; i++) {
      m_ledBuffer.setLED(i, color);
    }

    m_led.setData(m_ledBuffer);
    leftLEDsOn = true;
  }

  public void setColorRight(Color color) {
    for (int i = LEDS.STRIP_LEFT_LENGTH; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }

    m_led.setData(m_ledBuffer);
    rightLEDsOn = true;
  }

  public void toggleColor(Color color) {
    if (areLEDsOn()) {
      stop();
    } else {
      setColor(color);
    }
  }

  public void toggleColorLeft(Color color) {
    if (leftLEDsOn) {
      stop();
    } else {
      setColorLeft(color);
    }
  }

  public void toggleColorRight(Color color) {
    if (leftLEDsOn) {
      stop();
    } else {
      setColorRight(color);
    }
  }

  public void stop() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, new Color());
    }

    m_led.setData(m_ledBuffer);
    leftLEDsOn = false;
    rightLEDsOn = false;
  }

  public void stopLeft() {
    for (int i = 0; i < LEDS.STRIP_LEFT_LENGTH; i++) {
      m_ledBuffer.setLED(i, new Color());
    }

    m_led.setData(m_ledBuffer);
    leftLEDsOn = false;
  }

  public void stopRight() {
    for (int i = LEDS.STRIP_LEFT_LENGTH; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, new Color());
    }

    m_led.setData(m_ledBuffer);
    rightLEDsOn = false;
  }

  public boolean areLEDsOn() {
    return leftLEDsOn | rightLEDsOn;
  }

  public boolean areLeftLEDsOn() {
    return leftLEDsOn;
  }

  public boolean areRightLEDsOn() {
    return rightLEDsOn;
  }
}
