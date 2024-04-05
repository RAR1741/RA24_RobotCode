package frc.robot.constants;

public class VisionConstants {
  // Vision constants
  public int minTagCount;
  public double maxAvgDistance;
  public double autoStdDevScale;
  public double autoTranslationMax;

  public VisionConstants(int minTagCount, double maxAvgDistance, double autoStdDevScale, double autoTranslationMax) {
    this.minTagCount = minTagCount;
    this.maxAvgDistance = maxAvgDistance;
    this.autoStdDevScale = autoStdDevScale;
    this.autoTranslationMax = autoTranslationMax;
  }
}
