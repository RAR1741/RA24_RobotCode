package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDModes {
  private static final double k_scale = 0.5;
  
  private static final double hexToDouble(int sixteens, int ones) {return ((double) (16 * sixteens + ones)) / 256.0;}
  private static final double[] pacerBlue = {0.0, hexToDouble(2, 13), hexToDouble(6, 2)}; // #002D62
  private static final double[] pacerYellow = {hexToDouble(15, 13), hexToDouble(11, 11), hexToDouble(3, 0)}; // #FDBB30
  
  private static final void setLEDPacerBlue(AddressableLEDBuffer buffer, int i, int brightness) {
    buffer.setRGB(i, (int) (pacerBlue[0] * (double) brightness), (int) (pacerBlue[1] * (double) brightness), (int) (pacerBlue[2] * (double) brightness));
  }
  private static final void setLEDPacerYellow(AddressableLEDBuffer buffer, int i, int brightness) {
    buffer.setRGB(i, (int) (pacerYellow[0] * (double) brightness), (int) (pacerYellow[1] * (double) brightness), (int) (pacerYellow[2] * (double) brightness));
  }

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> setColor(
      Color color) {
    return (
        start) -> {
      return (length) -> {
        return (buffer) -> {
          Color newColor = color;

          if (color == Color.kYellow) {
            // Custom yellow
            newColor = new Color(255, (int) (255 * 0.50), 0);
          }
          
          for (int i = start; i < (start + length); i++) {
            buffer.setLED(i, scaleColor(newColor));
          }
          return buffer;
        };
      };
    };
  }

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> rainbowChase = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double rainbowChaseSpeed = 100;

        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * rainbowChaseSpeed) % 180);
        for (int i = start; i < (start + length); i++) {
          final int hue = (firstPixelHue + (i * 180 / length)) % 180;
          buffer.setHSV(i, hue, 255, (int) (128*k_scale));
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> rainbowBreatheSlow = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double rainbowBreathSpeedSlow = 50;

        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * rainbowBreathSpeedSlow) % 180);
        for (int i = start; i < (start + length); i++) {
          buffer.setHSV(i, firstPixelHue, 255, (int) (128*k_scale));
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> rainbowBreatheFast = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double rainbowBreathSpeedFast = 500;

        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * rainbowBreathSpeedFast) % 180);
        for (int i = start; i < (start + length); i++) {
          buffer.setHSV(i, firstPixelHue, 255, (int) (128*k_scale));
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> redChase = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double chaseSpeed = 100;

        int firstPixelBrightness = (int) ((System.currentTimeMillis() / 1000.0 * chaseSpeed) % 360);
        for (int i = start; i < (start + length); i++) {
          final int brightness = (firstPixelBrightness + (i * 180 / length)) % 360 + 10;
          if (brightness < 190) {
            setLEDPacerBlue(buffer, i, brightness);
          } else {
            setLEDPacerYellow(buffer, i, brightness - 180);
          }
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> redBreathe = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double breatheSpeed = 205;

        int brightness = (int) (Math.pow(Math.sin(System.currentTimeMillis() / 1000.0), 2) * breatheSpeed) + 50; // sin^2 period is 3.14 = π seconds
        for (int i = start; i < (start + length); i++) {
          if ((System.currentTimeMillis() / 1000.0) % (2.0 * Math.PI) < Math.PI) {
            setLEDPacerBlue(buffer, i, brightness);
          } else {
            setLEDPacerYellow(buffer, i, brightness);
          }
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> redTwinkleSlow = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double twinkleRate = 0.001;
        int reduction = 5;

        for (int i = start; i < (start + length); i++) {
          // Get the LED at the current index
          Color led = buffer.getLED(i);
          int r = (int) (led.red * 255);

          // If its red component is greater than 0, then we want to decrease it
          if (r > 0 && Math.random() < 1.0) {
            r -= reduction;
          } else {
            // Otherwise, at a random chance, set it to full red
            if (Math.random() < twinkleRate) {
              r = 255;
            }
          }
          r = MathUtil.clamp(r, 0, (int) (255*k_scale));
          buffer.setRGB(i, r, 0, 0);
        }
        return buffer;
      };
    };
  };

  public static Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> redTwinkleFast = (
      start) -> {
    return (length) -> {
      return (buffer) -> {
        double twinkleRate = 0.01;
        int reduction = 1;

        for (int i = start; i < (start + length); i++) {
          // Get the LED at the current index
          Color led = buffer.getLED(i);
          int r = (int) (led.red * 255);

          // If its red component is greater than 0, then we want to decrease it
          if (r > 0 && Math.random() < 1.0) {
            r -= reduction;
          } else {
            // Otherwise, at a random chance, set it to full red
            if (Math.random() < twinkleRate) {
              r = 255;
            }
          }
          r = MathUtil.clamp(r, 0, (int) (255*k_scale));
          buffer.setRGB(i, r, 0, 0);
        }
        return buffer;
      };
    };
  };

  public static Color scaleColor(Color oldColor){
    Color newColor = oldColor;
    
    if (oldColor == Color.kYellow) {
      // Custom yellow
      newColor = new Color(255, (int) (255 * 0.50), 0);
    }

    return new Color(newColor.red*k_scale, newColor.green*k_scale, newColor.blue*k_scale);
  }
}
