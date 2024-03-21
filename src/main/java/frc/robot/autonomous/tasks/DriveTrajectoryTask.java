package frc.robot.autonomous.tasks;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ApolloConstants.Auto;
import frc.robot.constants.ApolloConstants.AutoAim.Rotation;
import frc.robot.constants.ApolloConstants.AutoAim.Translation;
import frc.robot.constants.ApolloConstants.Robot;
import frc.robot.subsystems.drivetrain.RARHolonomicDriveController;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveTrajectoryTask extends Task {
  private SwerveDrive m_swerve = SwerveDrive.getInstance();
  private PathPlannerTrajectory m_autoTrajectory;
  private PathPlannerPath m_autoPath = null;

  private RARHolonomicDriveController k_driveController = new RARHolonomicDriveController(
      new PIDConstants(Translation.k_P, Translation.k_I, Translation.k_D),
      new PIDConstants(Rotation.k_P, Rotation.k_I, Rotation.k_D),
      Auto.k_maxModuleSpeed,
      Units.inchesToMeters(Math.sqrt(2) * (Robot.k_width / 2)));

  private final Timer m_runningTimer = new Timer();

  public DriveTrajectoryTask(String pathName) {
    try {
      m_autoPath = PathPlannerPath.fromPathFile(pathName);

      if (DriverStation.getAlliance().get() == Alliance.Red) {
        DriverStation.reportWarning("Translating path for Red Alliance!", false);
        m_autoPath = m_autoPath.flipPath();
      }

    } catch (Exception ex) {
      DriverStation.reportError("Unable to load PathPlanner trajectory: " + pathName, ex.getStackTrace());
    }
  }

  @Override
  public void start() {
    DriverStation.reportWarning("Auto trajectory start", false);
    m_runningTimer.reset();
    m_runningTimer.start();

    // We probably want to reset this to the pose's starting rotation
    m_swerve.setAllianceGyroAngleAdjustment();

    m_autoTrajectory = m_autoPath.getTrajectory(
        new ChassisSpeeds(),
        m_swerve.getGyro().getRotation2d());

    ArrayList<Pose2d> newStates = new ArrayList<>();
    for (State state : m_autoTrajectory.getStates()) {
      newStates.add(state.getTargetHolonomicPose());
    }

    Trajectory adjustedTrajectory = TrajectoryGenerator.generateTrajectory(
        newStates,
        new TrajectoryConfig(
            m_autoPath.getGlobalConstraints().getMaxVelocityMps(),
            m_autoPath.getGlobalConstraints().getMaxAccelerationMpsSq()));

    Logger.recordOutput("Auto/DriveTrajectory/TargetTrajectory", adjustedTrajectory);
    Logger.recordOutput("Auto/DriveTrajectory/StartingTargetPose", getStartingPose());

    // TODO: we probably want to do this all the time?
    if (!m_swerve.hasSetPose()) {
      m_swerve.resetOdometry(getStartingPose());
    }

    m_swerve.clearTurnPIDAccumulation();
    DriverStation.reportWarning("Running path for " + DriverStation.getAlliance().get().toString(), false);
  }

  @Override
  public void update() {
    log(true);

    if (m_autoTrajectory != null) {
      State goal = m_autoTrajectory.sample(m_runningTimer.get());
      Pose2d pose = m_swerve.getPose();

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
    // if (!RobotBase.isReal() && m_autoTrajectory != null) {
    // m_swerve.setPose(m_autoTrajectory.sample(m_runningTimer.get()).getTargetHolonomicPose());
    // Pose2d pose =
    // m_autoTrajectory.sample(m_runningTimer.get()).getTargetHolonomicPose();

    // Preferences.setDouble("SwerveDrive/x", pose.getX());
    // Preferences.setDouble("SwerveDrive/y", pose.getY());
    // Preferences.setDouble("SwerveDrive/rot", pose.getRotation().getDegrees());
    // }
  }

  public Pose2d getStartingPose() {
    return m_autoTrajectory.getState(0).getTargetHolonomicPose();
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }

  @Override
  public void done() {
    log(false);

    DriverStation.reportWarning("Auto trajectory done", false);
    m_swerve.drive(0, 0, 0, true);
    // m_swerve.setRotationTarget(m_swerve.getPose().getRotation());
  }
}
