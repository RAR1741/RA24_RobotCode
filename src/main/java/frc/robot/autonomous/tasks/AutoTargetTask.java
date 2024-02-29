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
    return m_chassisSpeeds.vxMetersPerSecond;
  }

  @AutoLogOutput(key = "Auto/AutoTarget/getChassisY")
  public double getChassisY() {
    return m_chassisSpeeds.vyMetersPerSecond;
  }

  @AutoLogOutput(key = "Auto/AutoTarget/getChassisTheta")
  public double getChassisTheta() {
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

    System.out.println("Target degrees: " + Helpers.modDegrees(m_targetRotation.getDegrees()));
    m_targetRobotPose = new Pose2d(currentPose.getTranslation(), m_targetRotation);
  }

  @Override
  public void update() {
    double rotationError = Helpers.modRadians(m_targetRobotPose.getRotation().getRadians())
        - Helpers.modRadians(m_swerve.getPose().getRotation().getRadians());

    m_swerve.drive(0, 0, rotationError * 5.0, true);
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
        Helpers.modDegrees(m_swerve.getPose().getRotation().getDegrees())
            - Helpers.modDegrees(m_targetRobotPose.getRotation().getDegrees()));

    return m_isFinished || (rotationError < Constants.AutoAim.k_autoAimAngleTolerance) || !RobotBase.isReal();
  }

  @Override
  public void done() {
    DriverStation.reportWarning("Auto target done", false);
    m_swerve.drive(0, 0, 0, true);
  }
}
