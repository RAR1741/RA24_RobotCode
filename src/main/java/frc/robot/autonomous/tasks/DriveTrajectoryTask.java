package frc.robot.autonomous.tasks;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveTrajectoryTask extends Task {
  private SwerveDrive m_swerve = SwerveDrive.getInstance();;
  private PathPlannerTrajectory m_autoTrajectory;
  private boolean m_isFinished = false;
  private String m_smartDashboardKey = "DriveTrajectoryTask/";

  private final Timer m_runningTimer = new Timer();
  private PPHolonomicDriveController m_driveController;

  public DriveTrajectoryTask(String pathName) {
    try {
      PathPlannerPath m_autoPath = PathPlannerPath.fromPathFile(pathName);
      m_autoTrajectory = m_autoPath.getTrajectory(new ChassisSpeeds(0,0,0), new Rotation2d(0));
      // m_autoTrajectory = new PathPlannerTrajectory(
      //     m_autoPath,
      //     new ChassisSpeeds(0,0,0),
      //     m_swerve.getPose().getRotation());

    } catch (Exception ex) {
      DriverStation.reportError("Unable to load PathPlanner trajectory: " + pathName, ex.getStackTrace());
      m_isFinished = true;
    }

    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
    m_driveController = new PPHolonomicDriveController(
      new PIDConstants(0.5, 0, 0), 
      new PIDConstants(1,0, 0),
      Constants.SwerveDrive.k_maxSpeed, 
      Constants.Robot.k_width/2);
  }

  @Override
  public void start() {
    m_runningTimer.reset();
    m_runningTimer.start();

    m_swerve.clearTurnPIDAccumulation();
    DriverStation.reportWarning("Running path for " + DriverStation.getAlliance().toString(), false);
  }

  @Override
  public void update() {
    State goal = m_autoTrajectory.sample(m_runningTimer.get());
    ChassisSpeeds chassisSpeeds = m_driveController.calculateRobotRelativeSpeeds(m_swerve.getPose(), goal);


    m_swerve.drive(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        false);

    m_isFinished |= m_runningTimer.get() >= m_autoTrajectory.getTotalTimeSeconds();

    SmartDashboard.putNumber(m_smartDashboardKey + "vx", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "vy", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "vr", chassisSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void updateSim() {
    if (!RobotBase.isReal()) {
      PathPlannerTrajectory.State autoState = (PathPlannerTrajectory.State) m_autoTrajectory
          .sample(m_runningTimer.get());

      Pose2d targetPose2d = new Pose2d(
          autoState.positionMeters.getX(),
          autoState.positionMeters.getY(),
          autoState.targetHolonomicRotation);

      m_swerve.setPose(targetPose2d);
    }
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
