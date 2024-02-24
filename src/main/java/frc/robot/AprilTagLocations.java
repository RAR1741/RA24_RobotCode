package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class AprilTagLocations {
  private static final Transform3d[] aprilTagPositions = {
      null, // Index 0 is not used
      Red.k_sourceTag1,
      Red.k_sourceTag2,
      Red.k_speakerTag3,
      Red.k_speakerTag4,
      Red.k_ampTag5,
      Blue.k_ampTag6,
      Blue.k_speakerTag7,
      Blue.k_speakerTag8,
      Blue.k_sourceTag9,
      Blue.k_sourceTag10,
      Red.k_stageSourceSideTag11,
      Red.k_stageAmpSideTag12,
      Red.k_stageCenterSideTag13,
      Blue.k_stageCenterSideTag14,
      Blue.k_stageAmpSideTag15,
      Blue.k_stageSourceSideTag16
  };

  // Method to get AprilTag position by ID
  public static Transform3d getPosition(int id) {
    if (id >= 1 && id < aprilTagPositions.length) {
      Transform3d position = aprilTagPositions[id];
      if (position != null) {
        return position;
      }
    }
    DriverStation.reportError("AprilTag [" + id + "] is not valid", true);
    return null;
  }

  public static class Red {
    public static final Transform3d k_sourceTag1 = new Transform3d(
        new Translation3d(15.079472, 0.245872, 1.355852),
        new Rotation3d(0, 0, Units.degreesToRadians(120)));

    public static final Transform3d k_sourceTag2 = new Transform3d(
        new Translation3d(16.185134, 0.883666, 1.355852),
        new Rotation3d(0, 0, Units.degreesToRadians(120)));

    public static final Transform3d k_speakerTag3 = new Transform3d(
        new Translation3d(16.579342, 4.982718, 1.451102),
        new Rotation3d(0, 0, Units.degreesToRadians(180)));

    public static final Transform3d k_speakerTag4 = new Transform3d(
        new Translation3d(16.579342, 5.547868, 1.451102),
        new Rotation3d(0, 0, Units.degreesToRadians(180)));

    public static final Transform3d k_ampTag5 = new Transform3d(
        new Translation3d(14.700758, 8.2042, 1.355852),
        new Rotation3d(0, 0, Units.degreesToRadians(270)));

    public static final Transform3d k_stageSourceSideTag11 = new Transform3d(
        new Translation3d(11.904726, 3.713226, 1.3208),
        new Rotation3d(0, 0, Units.degreesToRadians(300)));

    public static final Transform3d k_stageAmpSideTag12 = new Transform3d(
        new Translation3d(11.904726, 4.49834, 1.3208),
        new Rotation3d(0, 0, Units.degreesToRadians(60)));

    public static final Transform3d k_stageCenterSideTag13 = new Transform3d(
        new Translation3d(11.220196, 4.105148, 1.3208),
        new Rotation3d(0, 0, Units.degreesToRadians(180)));
  }

  public static class Blue {
    public static final Transform3d k_sourceTag9 = new Transform3d(
        new Translation3d(0.356108, 0.883666, 1.355852),
        new Rotation3d(0, 0, Units.degreesToRadians(60)));

    public static final Transform3d k_sourceTag10 = new Transform3d(
        new Translation3d(1.461516, 0.245872, 1.355852),
        new Rotation3d(0, 0, Units.degreesToRadians(60)));

    public static final Transform3d k_speakerTag7 = new Transform3d(
        new Translation3d(-0.0381, 5.547868, 1.451102),
        new Rotation3d(0, 0, Units.degreesToRadians(0)));

    public static final Transform3d k_speakerTag8 = new Transform3d(
        new Translation3d(-0.0381, 4.982718, 1.451102),
        new Rotation3d(0, 0, Units.degreesToRadians(0)));

    public static final Transform3d k_ampTag6 = new Transform3d(
        new Translation3d(1.8415, 8.2042, 1.355852),
        new Rotation3d(0, 0, Units.degreesToRadians(270)));

    public static final Transform3d k_stageCenterSideTag14 = new Transform3d(
        new Translation3d(5.320792, 4.105148, 1.3208),
        new Rotation3d(0, 0, Units.degreesToRadians(0)));

    public static final Transform3d k_stageAmpSideTag15 = new Transform3d(
        new Translation3d(4.641342, 4.49834, 1.3208),
        new Rotation3d(0, 0, Units.degreesToRadians(120)));

    public static final Transform3d k_stageSourceSideTag16 = new Transform3d(
        new Translation3d(4.641342, 3.713226, 1.3208),
        new Rotation3d(0, 0, Units.degreesToRadians(240)));
  }
}
