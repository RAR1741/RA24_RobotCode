package frc.robot.autonomous.tasks;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveTrajectoryTask extends Task {
  private SwerveDrive m_swerve = SwerveDrive.getInstance();;
  private PathPlannerTrajectory m_autoTrajectory;
  private boolean m_isFinished = false;
  private PathPlannerPath m_autoPath = null;
  private String m_smartDashboardKey = "DriveTrajectoryTask/";

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
        Constants.SwerveDrive.k_maxSpeed,
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
      Logger.recordOutput("Auto/DriveTrajectory/TargetPose", goal.getTargetHolonomicPose());

      ChassisSpeeds chassisSpeeds = m_driveController.calculateRobotRelativeSpeeds(m_swerve.getPose(), goal);

      m_swerve.drive(
          chassisSpeeds.vxMetersPerSecond,
          chassisSpeeds.vyMetersPerSecond,
          chassisSpeeds.omegaRadiansPerSecond,
          false); // TODO: figure out if this is correct

      m_isFinished |= m_runningTimer.get() >= m_autoTrajectory.getTotalTimeSeconds();

      SmartDashboard.putNumber(m_smartDashboardKey + "vx", chassisSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber(m_smartDashboardKey + "vy", chassisSpeeds.vyMetersPerSecond);
      SmartDashboard.putNumber(m_smartDashboardKey + "vr", chassisSpeeds.omegaRadiansPerSecond);
    } else {
      m_isFinished = true;
    }
  }

  @Override
  public void updateSim() {
    if (!RobotBase.isReal() && m_autoTrajectory != null) {
      State state = m_autoTrajectory.sample(m_runningTimer.get());

      Preferences.setDouble("SwerveDrive/x", state.getTargetHolonomicPose().getX());
      Preferences.setDouble("SwerveDrive/y", state.getTargetHolonomicPose().getY());
      Preferences.setDouble("SwerveDrive/rot", state.getTargetHolonomicPose().getRotation().getDegrees());
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
