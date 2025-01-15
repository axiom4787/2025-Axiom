package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Leds {
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;

  int[] coralColor = {255, 255, 255};
  int[] algaeColor = {0, 255, 255};

  public Leds(int port, int length) {
    m_led = new AddressableLED(port);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(m_ledBuffer.getLength());
  }
}