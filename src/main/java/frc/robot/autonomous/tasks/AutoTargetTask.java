package frc.robot.autonomous.tasks;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Helpers;
import frc.robot.RobotTelemetry;
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
    RobotTelemetry.print("Auto target start");

    Pose2d currentPose = m_swerve.getPose();
    RobotTelemetry.print("Starting degrees: " + Helpers.modDegrees(currentPose.getRotation().getDegrees()));

    // Get the target rotation between the current pose and the target pose
    m_targetRotation = new Rotation2d(
        currentPose.getTranslation().getX() - m_targetPose.getTranslation().getX(),
        currentPose.getTranslation().getY() - m_targetPose.getTranslation().getY());

    m_targetRobotPose = new Pose2d(currentPose.getTranslation(), m_targetRotation);
  }

  @Override
  public void update() {
    log(true);
    m_swerve.driveLockedHeading(0, 0, 0, true, true, false, false);
  }

  @Override
  public void updateSim() {
    if (!RobotBase.isReal()) {
      m_swerve.setPose(m_targetRobotPose);
    }
  }

  @Override
  public boolean isFinished() {
    return m_swerve.isAimedAtTarget() || !RobotBase.isReal();
  }

  @Override
  public void done() {
    log(false);

    RobotTelemetry.print("Auto target done");
    m_swerve.drive(0, 0, 0, true);
  }
}
