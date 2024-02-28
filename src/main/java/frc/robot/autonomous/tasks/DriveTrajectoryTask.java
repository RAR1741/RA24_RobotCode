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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Auto;
import frc.robot.Constants.AutoAim.Rotation;
import frc.robot.Constants.AutoAim.Translation;
import frc.robot.Constants.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveTrajectoryTask extends Task {
  private SwerveDrive m_swerve = SwerveDrive.getInstance();
  private PathPlannerTrajectory m_autoTrajectory;
  private PathPlannerPath m_autoPath = null;

  private PPHolonomicDriveController k_driveController = new PPHolonomicDriveController(
      new PIDConstants(
          Translation.k_P,
          Translation.k_I,
          Translation.k_D),
      new PIDConstants(
          Rotation.k_P,
          Rotation.k_I,
          Rotation.k_D),
      Auto.k_maxModuleSpeed,
      Robot.k_width / 2);

  private final Timer m_runningTimer = new Timer();

  public DriveTrajectoryTask(String pathName) {
    try {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        DriverStation.reportWarning("Translating path for Red Alliance!", false);

        pathName = "RED|" + pathName;
      }

      m_autoPath = PathPlannerPath.fromPathFile(pathName);

    } catch (Exception ex) {
      DriverStation.reportError("Unable to load PathPlanner trajectory: " + pathName, ex.getStackTrace());
    }
  }

  @Override
  public void start() {
    m_runningTimer.reset();
    m_runningTimer.start();

    // We probably want to reset this to the pose's starting rotation
    m_swerve.setAllianceGyroAngleAdjustment();

    // m_autoTrajectory = m_autoPath.getTrajectory(
    // new ChassisSpeeds(),
    // m_swerve.getGyro().getRotation2d());

    m_autoTrajectory = m_autoPath.getTrajectory(
        new ChassisSpeeds(),
        m_swerve.getPose().getRotation());

    Logger.recordOutput("Auto/DriveTrajectory/StartingTargetPose", getStartingPose());

    // TODO: we probably want to do this all the time?
    // if (!m_swerve.hasSetPose()) {
    m_swerve.resetOdometry(getStartingPose());
    // }

    m_swerve.clearTurnPIDAccumulation();
    DriverStation.reportWarning("Running path for " + DriverStation.getAlliance().get().toString(), false);
  }

  @Override
  public void update() {
    if (m_autoTrajectory != null) {
      // Rotation2d offset = new Rotation2d(Units.degreesToRadians(180));

      State goal = m_autoTrajectory.sample(m_runningTimer.get());

      // goal.targetHolonomicRotation = Rotation2d.fromDegrees(180);
      // goal.targetHolonomicRotation = goal.targetHolonomicRotation.rotateBy(offset);

      Pose2d pose = m_swerve.getPose();
      // pose = pose.rotateBy(offset);

      // if (DriverStation.getAlliance().get() == Alliance.Red) {
      // pose = pose.rotateBy(new Rotation2d(180));
      // }

      // goal.targetHolonomicRotation = goal.targetHolonomicRotation.rotateBy(new
      // Rotation2d(2 * Math.PI));

      ChassisSpeeds chassisSpeeds = k_driveController.calculateRobotRelativeSpeeds(pose, goal);

      Logger.recordOutput("Auto/DriveTrajectory/TargetPose", goal.getTargetHolonomicPose());
      Logger.recordOutput("Auto/DriveTrajectory/BigDummyTargetPose", new Pose2d(
          goal.getTargetHolonomicPose().getX(),
          goal.getTargetHolonomicPose().getY(),
          goal.targetHolonomicRotation));

      Logger.recordOutput("Auto/DriveTrajectory/TargetHoloRotation",
          goal.targetHolonomicRotation);
      Logger.recordOutput("Auto/DriveTrajectory/TargetRotationDegrees",
          goal.getTargetHolonomicPose().getRotation().getDegrees());
      Logger.recordOutput("Auto/DriveTrajectory/ActualRotationDegrees", pose.getRotation().getDegrees());

      Logger.recordOutput("Auto/DriveTrajectory/RotationError",
          goal.getTargetHolonomicPose().getRotation().getDegrees() - pose.getRotation().getDegrees());
      Logger.recordOutput("Auto/DriveTrajectory/ChassisRotationDPS",
          Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));

      m_swerve.drive(chassisSpeeds);

      m_isFinished |= m_runningTimer.get() >= m_autoTrajectory.getTotalTimeSeconds();
    } else {
      m_isFinished = true;
    }
  }

  @Override
  public void updateSim() {
    if (!RobotBase.isReal() && m_autoTrajectory != null) {
      // Pose2d pose =
      // m_autoTrajectory.sample(m_runningTimer.get()).getTargetHolonomicPose();

      // Preferences.setDouble("SwerveDrive/x", pose.getX());
      // Preferences.setDouble("SwerveDrive/y", pose.getY());
      // Preferences.setDouble("SwerveDrive/rot", pose.getRotation().getDegrees());
    }
  }

  public Pose2d getStartingPose() {
    System.out.println(m_autoTrajectory.getState(0).getTargetHolonomicPose().getRotation().getDegrees());
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
