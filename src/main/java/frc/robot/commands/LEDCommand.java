package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants;

public class LEDCommand extends Command{

  private final LEDSubsystem m_LEDSubsystem;
  private Constants.LEDPresets state;

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public LEDCommand(LEDSubsystem subsystem, Constants.LEDPresets LEDstate) {
    m_LEDSubsystem = subsystem;
    addRequirements(m_LEDSubsystem);
    state = LEDstate;
  }

  @Override
  public void initialize() {

    //set up default patterns
    m_LEDSubsystem.setPattern(Constants.LEDPresets.LEDS_OFF, this::LedOff);
    m_LEDSubsystem.setPattern(Constants.LEDPresets.LEDS_RAINBOW, this::LedRainbow);
    m_LEDSubsystem.setPattern(Constants.LEDPresets.LEDS_TEAM_COLOR, this::LedTeamColor);

  }

  @Override
  public void execute() {

    m_LEDSubsystem.getPattern(state).run();
  }

  @Override
  public void end(boolean interrupted) {
    LedOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void LedTeamColor() {
    var m_buffer = m_LEDSubsystem.getBuffer();

    (switch (DriverStation.getAlliance().get()) {
      case Blue -> LEDPattern.solid(Color.kBlue); // (Blue: #0000FF)
      case Red -> LEDPattern.solid(Color.kRed); // (Red: #FF0000)
      default -> LEDPattern.solid(Color.kMagenta); // (Magenta: #FF00FF)
    }).applyTo(m_buffer);

    m_LEDSubsystem.setBuffer(m_buffer);
  }

  public void LedOff() {
    var m_buffer = m_LEDSubsystem.getBuffer();

    LEDPattern.kOff
        .applyTo(m_buffer);

    m_LEDSubsystem.setBuffer(m_buffer);
  }

  public void LedRainbow() {
    var m_buffer = m_LEDSubsystem.getBuffer();

    LEDPattern
    .rainbow(255,255)
    .scrollAtRelativeSpeed(Percent.per(Second).of(25))
    .applyTo(m_buffer);

    m_LEDSubsystem.setBuffer(m_buffer);
  }

}