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

      /**
       * putPattern
       * 
       * @param state Member of the LEDPresets enum to assign the pattern to
       * @param function Runnable object that sets the LED pattern, see LEDCommand.java for examples
       */
      public void putPattern(Constants.LEDPresets state, Runnable function) {
            LedMap.put(state, function);
      }

      /**
       * getPattern
       * 
       * @param state Member of the LEDPresets enum to get the pattern from
       * @return Runnable object that sets the LED pattern
       */
      public Runnable getPattern(Constants.LEDPresets state) {
            return LedMap.get(state);
      }

      /**
       * getBuffer
       * 
       * @return AddressableLEDBuffer object
       */
      public AddressableLEDBuffer getBuffer() {
            return buffer;
      }

      /*
       * setBuffer
       * 
       * @param buffer AddressableLEDBuffer object to set the buffer to
       */
      public void setBuffer(AddressableLEDBuffer buffer) {
            leds.setData(buffer);
      }

      /**
       * startLEDS
       */
      public void startLEDS() {
            leds.start();
      }

}