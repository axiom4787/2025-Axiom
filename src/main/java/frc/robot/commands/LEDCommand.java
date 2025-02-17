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
import frc.robot.Constants.RobotStates.States;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends Command{

  private Alliance allianceColor;
  private final LEDSubsystem m_LEDSubsystem;
  private final AddressableLEDBuffer m_buffer;
  private States state;

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public LEDCommand(LEDSubsystem subsystem, States LEDstate) {
    m_LEDSubsystem = subsystem;
    m_buffer = m_LEDSubsystem.getBuffer();
    allianceColor = Alliance.Blue;
    addRequirements(m_LEDSubsystem);
    state = LEDstate;
  }

  @Override
  public void initialize() {

    //set up default patterns
    m_LEDSubsystem.setPattern(States.LEDS_OFF, this::LedOff);
    m_LEDSubsystem.setPattern(States.LEDS_RAINBOW, this::LedRainbow);
    m_LEDSubsystem.setPattern(States.LEDS_TEAM_COLOR, this::LedTeamColor);

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
    (switch (DriverStation.getAlliance().get()) {
      case Blue -> LEDPattern.solid(Color.kBlue); // (Blue: #0000FF)
      case Red -> LEDPattern.solid(Color.kRed); // (Red: #FF0000)
      default -> LEDPattern.solid(Color.kMagenta); // (Magenta: #FF00FF)
    }).applyTo(m_buffer);

    m_LEDSubsystem.setBuffer(m_buffer);
  }

  public void LedOff() {
    LEDPattern.kOff
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

}