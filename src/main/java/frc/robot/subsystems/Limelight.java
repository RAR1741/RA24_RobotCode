package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Helpers;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Limelight {
  private NetworkTable m_limelightTable;
  private final String m_name;

  /**
   * Constructor
   */
  public Limelight(String limelightName) {
    m_name = limelightName;
    m_limelightTable = NetworkTableInstance.getDefault().getTable(m_name);
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

  public double getTimeOffset() {
    return Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Pipeline(m_name);
  }

  public void outputTelemetry() {
    for (String key : m_limelightTable.getKeys()) {
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

  public PoseEstimate getPoseEstimation() {
    LimelightHelpers.SetRobotOrientation("limelight",
        Helpers.modDegrees(SwerveDrive.getInstance().getGyro().getAngle()),
        0,
        // SwerveDrive.getInstance().getGyro().getRate(),
        0, 0, 0, 0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);
  }

  public double getLatency() {
    return LimelightHelpers.getLatency_Capture(m_name) + LimelightHelpers.getLatency_Pipeline(m_name);
  }

  public boolean getLightEnabled() {
    return m_limelightTable.getEntry("ledMode").getDouble(1) == 3;
  }
}
