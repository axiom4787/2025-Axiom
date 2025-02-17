package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants;

import java.util.HashMap;

public class LEDSubsystem extends SubsystemBase {
      private final AddressableLED leds;
      private final AddressableLEDBuffer buffer; // Creates a new buffer object
      private final HashMap<Constants.LEDPresets, Runnable> LedMap;

      /**
       * LEDSubsystem
       * 
       * @param port PWM port on the roboRIO
       */
      public LEDSubsystem(int port) {

            leds = new AddressableLED(port);
            leds.setLength(LEDConstants.BUFFER_LENGTH);
            buffer = new AddressableLEDBuffer(LEDConstants.BUFFER_LENGTH);
            LedMap = new HashMap<>();

            setBuffer(buffer);

            startLEDS();
      }

      public void setPattern(Constants.LEDPresets state, Runnable function) {
            LedMap.put(state, function);
      }

      public Runnable getPattern(Constants.LEDPresets state) {
            return LedMap.get(state);
      }

      public AddressableLEDBuffer getBuffer() {
            return buffer;
      }

      public void setBuffer(AddressableLEDBuffer buffer) {
            leds.setData(buffer);
      }

      public void startLEDS() {
            leds.start();
      }

}