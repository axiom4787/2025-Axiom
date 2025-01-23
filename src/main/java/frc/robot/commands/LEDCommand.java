package frc.robot.commands;


import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends Command{
  
  private Alliance allianceColor;

  private final LEDPatterns m_pattern;
  private final LEDSubsystem m_LEDSubsystem;

  private final AddressableLEDBuffer m_buffer;

  //TODO: Add more patterns and see if i can put everything in this enum 🤔
  public static enum LEDPatterns {
    OFF,
    RAINBOW,
    RSL_SYNC,
    TEAM_COLOR,

  };

    /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public LEDCommand(LEDSubsystem subsystem, LEDPatterns pattern) {
    m_pattern = pattern;
    m_LEDSubsystem = subsystem;
    m_buffer = m_LEDSubsystem.getBuffer();
    allianceColor = DriverStation.getAlliance().get();
    addRequirements(m_LEDSubsystem);
  }

  @Override
  public void initialize() {
    allianceColor = DriverStation.getAlliance().get();
  }

  @Override
  public void execute() {
    switch (m_pattern) {

      case RSL_SYNC -> LedRSLSync(); //syncronizes the LEDS with the RSL (Coral: #FF7F50)
      case TEAM_COLOR -> LedTeamColor(); //sets the LEDS to the team color
      case RAINBOW -> LedRainbow(); //sets the LEDS to a scrolling rainbow pattern  
      default -> LedOff(); //turns the LEDS off by default

    }
  }

  @Override
  public void end(boolean interrupted) {
    LedOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void LedRSLSync() {
    LEDPattern
    .solid(Color.kCoral)
    .synchronizedBlink(RobotController::getRSLState)
    .applyTo(m_buffer);

    m_LEDSubsystem.setBuffer(m_buffer);
  }

  public void LedRainbow() {
    LEDPattern
    .rainbow(255,255)
    .scrollAtRelativeSpeed(Percent.per(Second).of(25))
    .applyTo(m_buffer);

    m_LEDSubsystem.setBuffer(m_buffer);
  }

  public void LedTeamColor() {
    (switch (allianceColor) {
      case Blue -> LEDPattern.solid(Color.kBlue); // (Blue: #0000FF)
      case Red  -> LEDPattern.solid(Color.kRed); // (Red: #FF0000)
      default   -> LEDPattern.solid(Color.kMagenta); // (Magenta: #FF00FF)
    }).applyTo(m_buffer);

    m_LEDSubsystem.setBuffer(m_buffer);
  }

  public void LedOff() {
    LEDPattern
    .kOff
    .applyTo(m_buffer);

    m_LEDSubsystem.setBuffer(m_buffer);
  }

}