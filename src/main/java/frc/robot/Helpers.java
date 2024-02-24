package frc.robot;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

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

  public static Transform3d getAprilTagPosition(int id) {
    switch (id) {
      case 1:
        return AprilTagLocations.Red.k_sourceTag1;
      case 2:
        return AprilTagLocations.Red.k_sourceTag2;
      case 3:
        return AprilTagLocations.Red.k_speakerTag3;
      case 4:
        return AprilTagLocations.Red.k_speakerTag4;
      case 5:
        return AprilTagLocations.Red.k_ampTag5;
      case 6:
        return AprilTagLocations.Blue.k_ampTag6;
      case 7:
        return AprilTagLocations.Blue.k_speakerTag7;
      case 8:
        return AprilTagLocations.Blue.k_speakerTag8;
      case 9:
        return AprilTagLocations.Blue.k_sourceTag9;
      case 10:
        return AprilTagLocations.Blue.k_sourceTag10;
      case 11:
        return AprilTagLocations.Red.k_stageSourceSideTag11;
      case 12:
        return AprilTagLocations.Red.k_stageAmpSideTag12;
      case 13:
        return AprilTagLocations.Red.k_stageCenterSideTag13;
      case 14:
        return AprilTagLocations.Blue.k_stageCenterSideTag14;
      case 15:
        return AprilTagLocations.Blue.k_stageAmpSideTag15;
      case 16:
        return AprilTagLocations.Blue.k_stageSourceSideTag16;
      default: 
        DriverStation.reportError("AprilTag [" + id + "] is not valid", true);
        return null;
    }
  }
}
