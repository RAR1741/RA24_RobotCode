package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;

public class Limelight extends Subsystem {
  private static Limelight m_limelight;
  private NetworkTable m_limelightTable;
  private final String m_name = "limelight";

  /**
   * Constructor
   */
  private Limelight() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable(m_name);
  }

  /**
   * Get a new instance of the limelight class
   *
   * @return New instance of the limelight class
   */
  public static Limelight getInstance() {
    if (m_limelight == null) {
      m_limelight = new Limelight();
    }
    return m_limelight;
  }

  /**
   * Enable the LEDs
   */
  public void setLightEnabled(boolean enabled) {
    m_limelightTable.getEntry("ledMode").setNumber(enabled ? 3 : 1);
  }

  /**
   * Get the current bot position
   *
   * @return Current bot pose
   */
  public Pose2d getBotpose2D() {
    return toFieldPose(LimelightHelpers.getBotPose2d(m_name));
  }

  /**
   * Get whether there is a visible AprilTag
   *
   * @return If there is a visible AprilTag
   */
  public boolean seesAprilTag() {
    return m_limelightTable.getEntry("tv").getInteger(0) == 1;
  }

  public double getTimeOffset(double currentTime) {
    return currentTime - LimelightHelpers.getLatency_Pipeline(m_name);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void stop() {
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void outputTelemetry() {
    for (String key: m_limelightTable.getKeys()) {
      String type = m_limelightTable.getEntry(key).getType().name().substring(1);

      SmartDashboard.putString(
        key, (type.equals("String") || type.equals("Double")) ? m_limelightTable.getEntry(key).toString()
          : Arrays.toString(m_limelightTable.getEntry(key).getDoubleArray(new double[6])));
    }
  }

  /**
   * Converts the limelight coordinate system to the field coordinate system.
   *
   * @param pose Position of the robot
   * @return The position of the robot in terms of the field.
   */
  private Pose2d toFieldPose(Pose2d pose) {
    return pose.relativeTo(new Pose2d(-8.2296, -8.2296 / 2, Rotation2d.fromDegrees(0)));
  }
}
