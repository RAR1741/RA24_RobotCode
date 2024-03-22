package frc.robot.constants;

public class VisionConstants {
  // Vision constants
  public int minTagCount;
  public double maxAvgDistance;
  public double autoStdDevScale;

  public VisionConstants(int minTagCount, double maxAvgDistance, double autoStdDevScale) {
    this.minTagCount = minTagCount;
    this.maxAvgDistance = maxAvgDistance;
    this.autoStdDevScale = autoStdDevScale;
  }
}
