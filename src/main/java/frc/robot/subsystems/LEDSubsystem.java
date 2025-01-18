package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
      private final AddressableLED leds;
      private final AddressableLEDBuffer buffer;  // Creates a new buffer object

      /**
       * LEDSubsystem
       * @param port PWM port on the roboRIO
       */
      public LEDSubsystem(int port) {
            
            leds = new AddressableLED(port);
            leds.setLength(LEDConstants.kBufferLength);

            buffer = new AddressableLEDBuffer(LEDConstants.kBufferLength);
            
            setBuffer(buffer);

            leds.start();
      }

      public AddressableLEDBuffer getBuffer() {
            return buffer;
      }

      public void setBuffer(AddressableLEDBuffer buffer) {
            leds.setData(buffer);
      }

}