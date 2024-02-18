package frc.robot.autonomous.tasks;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveTrajectoryTask extends Task {
  private SwerveDrive m_swerve = SwerveDrive.getInstance();;
  private PathPlannerTrajectory m_autoTrajectory;
  private boolean m_isFinished = false;
  private PathPlannerPath m_autoPath = null;

  private final Timer m_runningTimer = new Timer();
  private PPHolonomicDriveController m_driveController;

  public DriveTrajectoryTask(String pathName) {
    try {
      m_autoPath = PathPlannerPath.fromPathFile(pathName);

      if (DriverStation.getAlliance().get() == Alliance.Red) {
        DriverStation.reportWarning("Translating path for Red Alliance!", false);
        m_autoPath = m_autoPath.flipPath();
        m_autoPath.preventFlipping = true;
      }

      m_autoTrajectory = m_autoPath.getTrajectory(
          new ChassisSpeeds(),
          m_swerve.getRotation2d());

    } catch (Exception ex) {
      DriverStation.reportError("Unable to load PathPlanner trajectory: " + pathName, ex.getStackTrace());
    }

    m_driveController = new PPHolonomicDriveController(
        new PIDConstants(
            Constants.Auto.PIDConstants.Translation.k_P,
            Constants.Auto.PIDConstants.Translation.k_I,
            Constants.Auto.PIDConstants.Translation.k_D),
        new PIDConstants(
            Constants.Auto.PIDConstants.Rotation.k_P,
            Constants.Auto.PIDConstants.Rotation.k_I,
            Constants.Auto.PIDConstants.Rotation.k_D),
        Constants.Auto.k_maxModuleSpeed,
        Constants.Robot.k_width / 2);
  }

  @Override
  public void start() {
    m_runningTimer.reset();
    m_runningTimer.start();

    m_swerve.clearTurnPIDAccumulation();
    DriverStation.reportWarning("Running path for " + DriverStation.getAlliance().get().toString(), false);
  }

  @Override
  public void update() {
    if (m_autoTrajectory != null) {
      State goal = m_autoTrajectory.sample(m_runningTimer.get());
      ChassisSpeeds chassisSpeeds = m_driveController.calculateRobotRelativeSpeeds(m_swerve.getPose(), goal);

      Logger.recordOutput("Auto/DriveTrajectory/TargetPose", goal.getTargetHolonomicPose());

      Logger.recordOutput("Auto/DriveTrajectory/RotationError",
          goal.getTargetHolonomicPose().getRotation().getDegrees() - m_swerve.getPose().getRotation().getDegrees());
      Logger.recordOutput("Auto/DriveTrajectory/ChassisRotationRPS",
          Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));

      // m_swerve.drive(
      // chassisSpeeds.vxMetersPerSecond,
      // chassisSpeeds.vyMetersPerSecond,
      // chassisSpeeds.omegaRadiansPerSecond,
      // true); // TODO: figure out if this is correct

      m_swerve.drive(chassisSpeeds);

      m_isFinished |= m_runningTimer.get() >= m_autoTrajectory.getTotalTimeSeconds();
    } else {
      m_isFinished = true;
    }
  }

  @Override
  public void updateSim() {
    if (!RobotBase.isReal() && m_autoTrajectory != null) {
      Pose2d pose = m_autoTrajectory.sample(m_runningTimer.get()).getTargetHolonomicPose();

      Preferences.setDouble("SwerveDrive/x", pose.getX());
      Preferences.setDouble("SwerveDrive/y", pose.getY());
      Preferences.setDouble("SwerveDrive/rot", pose.getRotation().getDegrees());
    }
  }

  public Pose2d getStartingPose() {
    // return m_autoPath.getPreviewStartingHolonomicPose();
    return m_autoTrajectory.getState(0).getTargetHolonomicPose();
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }

  @Override
  public void done() {
    DriverStation.reportWarning("Auto trajectory done", false);
    m_swerve.drive(0, 0, 0, true);
  }
}
