package frc.robot;

import com.revrobotics.CANSparkBase;

public class Helpers {
  public static double modRotations(double input) {
    input %= 1.0;
    if (input < 0.0) {
      input += 1.0;
    }
    return input;
  }

  public static double modRadians(double input) {
    input %= (2.0 * Math.PI);
    if (input < 0.0) {
      input += (2.0 * Math.PI);
    }
    return input;
  }

  public static double modDegrees(double input) {
    input %= 360.0;
    if (input < 0.0) {
      input += 360.0;
    }
    return input;
  }

  public static double getVoltage(CANSparkBase motor) {
    return motor.getBusVoltage() * motor.getAppliedOutput();
  }
}
