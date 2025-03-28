package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

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

        //default entries
        putPattern(Constants.LEDPresets.LEDS_OFF, this::LedOff);
        putPattern(Constants.LEDPresets.LEDS_RAINBOW, this::LedRainbow);
        putPattern(Constants.LEDPresets.LEDS_TEAM_COLOR, this::LedTeamColor);
        putPattern(Constants.LEDPresets.LEDS_RSL, this::LedRSL);

        startLEDS();
    }

    /**
     * putPattern
     * 
     * @param state    Member of the LEDPresets enum to assign the pattern to
     * @param function Runnable object that sets the LED pattern, see
     *                 LEDSubsystem.java for examples
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
     * gets the buffer used to store LED data
     * 
     * @return AddressableLEDBuffer object
     */
    public AddressableLEDBuffer getBuffer() {
        return buffer;
    }

    /**
     * setBuffer
     * 
     * @param buffer AddressableLEDBuffer object to set the buffer to
     */
    public void setBuffer(AddressableLEDBuffer buffer) {
        leds.setData(buffer);
    }

    /**
     * startLEDS
     * 
     * starts the LEDs
     */
    public void startLEDS() {
        leds.start();
    }

    /**
     * stopLEDS
     * 
     * stops the LEDs
     */
    public void stopLEDS() {
        leds.stop();
    }

    private void LedTeamColor() {
        var m_buffer = getBuffer();

        (switch (DriverStation.getAlliance().get()) {
            case Blue -> LEDPattern.solid(Color.kBlue); // (Blue: #0000FF)
            case Red -> LEDPattern.solid(Color.kRed); // (Red: #FF0000)
            default -> LEDPattern.solid(Color.kMagenta); // (Magenta: #FF00FF)
        }).applyTo(m_buffer);

        setBuffer(m_buffer);
    }

    private void LedOff() {
        var m_buffer = getBuffer();

        LEDPattern
                .solid(Color.kBlack)
                .applyTo(m_buffer);

        setBuffer(m_buffer);
    }

    private void LedRainbow() {
        var m_buffer = getBuffer();

        LEDPattern
                .rainbow(255, 255)
                .scrollAtRelativeSpeed(Percent.per(Second).of(25))
                .applyTo(m_buffer);

        setBuffer(m_buffer);
    }

    private void LedRSL() {
        var m_buffer = getBuffer();

        LEDPattern
                .solid(Color.kCoral)
                .synchronizedBlink(RobotController::getRSLState);

        setBuffer(m_buffer);
    }
}