package frc.robot;

public class Constants {
  public class LEDConstants {
    // TODO: change these values to match the LED strip
    public static final int LED_PORT = 0;
    public static final int BUFFER_LENGTH = 60;
    public static final double LED_SPACING = 1 / 120.0;
  }

  
  public static enum RobotStates{
    INTAKING,
    SCORING,
    CLIMBING,
    IDLE,
    //default states that dont depend on other mechanisms
    LEDS_RAINBOW,
    LEDS_OFF, 
    LEDS_TEAM_COLOR,
  };
}