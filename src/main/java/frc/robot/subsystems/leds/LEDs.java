package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Subsystem;

public class LEDs extends Subsystem {
  private static LEDs m_instance;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_buffer;

  private int m_ledTotalLength = RobotConstants.config.LEDs.k_totalLength;

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
      m_led = new AddressableLED(RobotConstants.config.LEDs.k_PWMId);
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
    if (Robot.k_ledsEnabled) {
      m_shooterColor = mode;
      m_driveColor = mode;
    }
  }

  public void setAllColor(Color color) {
    if (Robot.k_ledsEnabled) {
      setShooterColor(color);
      setDriveColor(color);
    }
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
    if (Robot.k_ledsEnabled) {
      setAllColorMode(LEDModes.redChase);
    }
  }

  public void breathe() {
    if (Robot.k_ledsEnabled) {
      setAllColorMode(LEDModes.redBreathe);
    }
  }

  public void rainbowChase() {
    if (Robot.k_ledsEnabled) {
      setAllColorMode(LEDModes.rainbowChase);
    }
  }

  public void rainbowBreatheSlow() {
    if (Robot.k_ledsEnabled) {
      setAllColorMode(LEDModes.rainbowBreatheSlow);
    }
  }

  public void rainbowBreatheFast() {
    if (Robot.k_ledsEnabled) {
      setAllColorMode(LEDModes.rainbowBreatheFast);
    }
  }

  public void redTwinkleSlow() {
    if (Robot.k_ledsEnabled) {
      setAllColorMode(LEDModes.redTwinkleSlow);
    }
  }

  public void redTwinkleFast() {
    if (Robot.k_ledsEnabled) {
      setAllColorMode(LEDModes.redTwinkleFast);
    }
  }

  public void off() {
    if (Robot.k_ledsEnabled) {
      setAllColorMode(LEDModes.setColor(Color.kBlack));
    }
  }

  public void setShooterColorMode() {
    m_buffer = m_shooterColor.apply(RobotConstants.config.LEDs.Shooter.k_start).apply(RobotConstants.config.LEDs.Shooter.k_length)
        .apply(m_buffer);
  }

  public void setDriveColorMode() {
    m_buffer = m_driveColor.apply(RobotConstants.config.LEDs.Drive.k_start).apply(RobotConstants.config.LEDs.Drive.k_length)
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
