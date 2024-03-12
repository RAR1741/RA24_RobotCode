package frc.robot.autonomous.tasks;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class AutoTargetTask extends Task {
  private SwerveDrive m_swerve = SwerveDrive.getInstance();

  // Where we want to aim
  private Pose2d m_targetRobotPose;
  private Pose3d m_targetPose;
  private ChassisSpeeds m_chassisSpeeds;
  private Rotation2d m_targetRotation;

  public AutoTargetTask(Pose3d targetPose) {
    m_targetPose = targetPose;

    Logger.recordOutput("Auto/AutoTarget/TargetPose", targetPose);
  }

  @AutoLogOutput(key = "Auto/AutoTarget/TargetRobotPose")
  public Pose2d getTargetRobotPose() {
    return m_targetRobotPose;
  }

  @AutoLogOutput(key = "Auto/AutoTarget/getChassisX")
  public double getChassisX() {
    Logger.recordOutput("Auto/AutoTarget/getChassisX", m_chassisSpeeds.vxMetersPerSecond);
    return m_chassisSpeeds.vxMetersPerSecond;
  }

  @AutoLogOutput(key = "Auto/AutoTarget/getChassisY")
  public double getChassisY() {
    Logger.recordOutput("Auto/AutoTarget/getChassisY", m_chassisSpeeds.vyMetersPerSecond);
    return m_chassisSpeeds.vyMetersPerSecond;
  }

  @AutoLogOutput(key = "Auto/AutoTarget/getChassisTheta")
  public double getChassisTheta() {
    Logger.recordOutput("Auto/AutoTarget/getChassisTheta", m_chassisSpeeds.omegaRadiansPerSecond);
    return m_chassisSpeeds.omegaRadiansPerSecond;
  }

  @Override
  public void start() {
    DriverStation.reportWarning("Auto target start", false);

    Pose2d currentPose = m_swerve.getPose();
    System.out.println("Starting degrees: " + Helpers.modDegrees(currentPose.getRotation().getDegrees()));

    // Get the target rotation between the current pose and the target pose
    m_targetRotation = new Rotation2d(
        currentPose.getTranslation().getX() - m_targetPose.getTranslation().getX(),
        currentPose.getTranslation().getY() - m_targetPose.getTranslation().getY());

    // System.out.println("Target degrees: " +
    // Helpers.modDegrees(m_targetRotation.getDegrees()));
    m_targetRobotPose = new Pose2d(currentPose.getTranslation(), m_targetRotation);
  }

  @Override
  public void update() {
    // Rotation2d diff = m_targetRotation.minus(m_swerve.getRotation2d());

    // m_swerve.drive(0, 0, diff.getRadians() * 5.0, true);

    m_swerve.driveLockedHeading(0, 0, 0, true, true, false);
  }

  @Override
  public void updateSim() {
    if (!RobotBase.isReal()) {
      m_swerve.setPose(m_targetRobotPose);
    }
  }

  @Override
  public boolean isFinished() {
    double rotationError = Math.abs(
        Helpers.modRadians(m_swerve.getPose().getRotation().getRadians())
            - Helpers.modRadians(m_swerve.getRotationTarget().getRadians()));

    Logger.recordOutput("Auto/AutoTarget/rotationError", rotationError);

    return ((rotationError < Constants.AutoAim.k_autoAimAngleTolerance) &&
        (Math.abs(m_swerve.getChassisSpeeds().omegaRadiansPerSecond) < Constants.AutoAim.k_autoAimOmegaRPSThreshold))
        || !RobotBase.isReal();
  }

  @Override
  public void done() {
    DriverStation.reportWarning("Auto target done", false);
    m_swerve.drive(0, 0, 0, true);
  }
}
