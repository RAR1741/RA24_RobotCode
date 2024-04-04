package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.constants.ApolloConstants;
import frc.robot.subsystems.Subsystem;

public class LEDs extends Subsystem {
  private static LEDs m_instance;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_buffer;

  private int m_ledTotalLength = ApolloConstants.LEDs.k_totalLength;

  // Main sections
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_shooterColor = LEDModes
      .setColor(Color.kRed);
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_driveColor = LEDModes.rainbowChase;

  public static LEDs getInstance() {
    if (m_instance == null) {
      m_instance = new LEDs();
    }
    return m_instance;
  }

  private LEDs() {
    super("LEDs");

    if (Robot.k_ledsEnabled) {
      m_led = new AddressableLED(ApolloConstants.LEDs.k_PWMId);
      m_led.setLength(m_ledTotalLength);
      m_buffer = new AddressableLEDBuffer(m_ledTotalLength);
      m_led.start();
    }
  }

  @Override
  public void periodic() {
    if (Robot.k_ledsEnabled) {
      setShooterColorMode();
      setDriveColorMode();

      m_led.setData(m_buffer);
    }
  }

  public void setAllColorMode(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> mode) {
    m_shooterColor = mode;
    m_driveColor = mode;
  }

  public void setAllColor(Color color) {
    setShooterColor(color);
    setDriveColor(color);
  }

  public void setShooterColor(Color color) {
    m_shooterColor = LEDModes.setColor(color);
  }

  public void setShooterColor(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> colorMode) {
    m_shooterColor = colorMode;
  }

  public void setDriveColor(Color color) {
    m_driveColor = LEDModes.setColor(color);
  }

  public void chase() {
    setAllColorMode(LEDModes.redChase);
  }

  public void breathe() {
    setAllColorMode(LEDModes.redBreathe);
  }

  public void rainbowChase() {
    setAllColorMode(LEDModes.rainbowChase);
  }

  public void rainbowBreatheSlow() {
    setAllColorMode(LEDModes.rainbowBreatheSlow);
  }

  public void rainbowBreatheFast() {
    setAllColorMode(LEDModes.rainbowBreatheFast);
  }

  public void redTwinkleSlow() {
    setAllColorMode(LEDModes.redTwinkleSlow);
  }

  public void redTwinkleFast() {
    setAllColorMode(LEDModes.redTwinkleFast);
  }

  public void off() {
    setAllColorMode(LEDModes.setColor(Color.kBlack));
  }

  public void setShooterColorMode() {
    m_buffer = m_shooterColor.apply(ApolloConstants.LEDs.Shooter.k_start).apply(ApolloConstants.LEDs.Shooter.k_length)
        .apply(m_buffer);
  }

  public void setDriveColorMode() {
    m_buffer = m_driveColor.apply(ApolloConstants.LEDs.Drive.k_start).apply(ApolloConstants.LEDs.Drive.k_length)
        .apply(m_buffer);
  }

  @Override
  public void stop() {
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void reset() {
  }
}
