package frc.robot.autonomous.tasks;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class AutoTargetTask extends Task {
  private SwerveDrive m_swerve = SwerveDrive.getInstance();

  // Where we want to aim
  private Pose2d m_targetRobotPose;
  private Pose3d m_targetPose;

  public AutoTargetTask(Pose3d targetPose) {
    m_targetPose = targetPose;

    Logger.recordOutput("Auto/AutoTarget/TargetPose", targetPose);
  }

  @Override
  public void start() {
    Pose2d currentPose = m_swerve.getPose();
    System.out.println("Starting degrees: " + currentPose.getRotation().getDegrees());

    // Get the target rotation between the current pose and the target pose
    Rotation2d targetRotation = new Rotation2d(
        currentPose.getTranslation().getX() - m_targetPose.getTranslation().getX(),
        currentPose.getTranslation().getY() - m_targetPose.getTranslation().getY());

    System.out.println("Target degrees: " + targetRotation.getDegrees());
    m_targetRobotPose = new Pose2d(currentPose.getTranslation(), targetRotation);
  }

  @Override
  public void update() {
    ChassisSpeeds chassisSpeeds = Constants.AutoAim.k_autoTargetController.calculate(
        m_swerve.getPose(),
        m_targetRobotPose,
        0.0,
        m_targetRobotPose.getRotation());

    m_swerve.drive(chassisSpeeds);
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
        m_swerve.getPose().getRotation().getDegrees()
            - m_targetRobotPose.getRotation().getDegrees());

    return m_isFinished || (rotationError < Constants.AutoAim.k_autoAimAngleTolerance);
  }

  @Override
  public void done() {
    DriverStation.reportWarning("Auto target done", false);
    m_swerve.drive(0, 0, 0, true);
  }
}
