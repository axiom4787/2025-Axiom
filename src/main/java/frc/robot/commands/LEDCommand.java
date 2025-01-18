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


  public static enum LEDPatterns {
    RSL_SYNC,
    TEAM_COLOR,
    OFF,
    RAINBOW,
  }

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
    LEDPattern pattern;
    switch (m_pattern) {
      //syncronizes the LEDS with the RSL (Coral: #FF7F50)
      case RSL_SYNC:
        pattern = LEDPattern.solid(Color.kCoral)
                            .synchronizedBlink(RobotController::getRSLState);
        break;

      //sets the LEDS to the team color
      case TEAM_COLOR:
        switch (allianceColor) {

          case Blue:
            pattern = LEDPattern.solid(Color.kBlue); // (Blue: #0000FF)
            break;

          case Red:
            pattern = LEDPattern.solid(Color.kRed); // (Red: #FF0000)
            break;

          default:
            pattern = LEDPattern.solid(Color.kMagenta); // (Magenta:rgb(255, 0, 255))

        }
        break;

      //sets the LEDS to a scrolling rainbow pattern  
      case RAINBOW:
        LEDPattern base = LEDPattern.rainbow(255,255);
        pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
        
        break;
        
      //turns the LEDS off by default
      default:
        pattern = LEDPattern.kOff;
        break;
    }

    //apply the pattern to the buffer
    pattern.applyTo(m_buffer);
    m_LEDSubsystem.setBuffer(m_buffer);
  }

  @Override
  public void end(boolean interrupted) {
    LEDPattern.kOff.applyTo(m_buffer);
    m_LEDSubsystem.setBuffer(m_buffer);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}