package frc.robot.subsystems;

import java.util.HashMap;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDPresets;

public final class LEDSubsystem extends SubsystemBase {
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer; // Creates a new buffer object
    private final HashMap<LEDPresets, Runnable> LedMap;
    private LEDPresets m_pattern;

    /**
     * LEDSubsystem
     * 
     * @param port PWM port on the roboRIO
     */
    public LEDSubsystem(int port) {

        m_pattern = LEDPresets.LEDS_OFF;
        leds = new AddressableLED(port);
        leds.setLength(LEDConstants.BUFFER_LENGTH);
        buffer = new AddressableLEDBuffer(LEDConstants.BUFFER_LENGTH);
        LedMap = new HashMap<>();

        putPattern(LEDPresets.LEDS_OFF, this::LedOff);
        putPattern(LEDPresets.LEDS_RAINBOW, this::LedRainbow);
        putPattern(LEDPresets.LEDS_TEAM_COLOR, this::LedTeamColor);
        putPattern(LEDPresets.LEDS_RSL, this::LedRSL);

        setBuffer(buffer);

        leds.start();
    }

    /**
     * putPattern
     * 
     * @param pattern  Member of the LEDPresets enum to assign the pattern to
     * @param function Runnable object that sets the LED pattern, see
     *                 LEDCommand.java for examples
     */
    public void putPattern(LEDPresets pattern, Runnable function) {
        LedMap.put(pattern, function);
    }

    /**
     * getPattern
     * 
     * @param pattern Member of the LEDPresets enum to get the pattern from
     * @return Runnable object that sets the LED pattern
     */
    public Runnable getPattern() {
        return LedMap.get(m_pattern);
    }

    /**
     * usePattern
     * 
     * @param pattern Tells the LEDCommand which preset to choose
     */

    public void usePattern(LEDPresets pattern) {
        m_pattern = pattern;
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

        LEDPattern.kOff
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
            .synchronizedBlink(RobotController::getRSLState)
            .applyTo(m_buffer);

        setBuffer(m_buffer);
    }

    public Command LEDCommand() {
        return new PrintCommand("LEDs Started!!");
    }

    @Override
    public void periodic() {
        System.out.println(m_pattern);
        getPattern().run();
    }
}