package frc.robot.autonomous.tasks;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.AllianceHelpers;
import frc.robot.RobotTelemetry;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotType;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.RARHolonomicDriveController;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveTrajectoryTask extends Task {
  private final boolean k_debugPath = false;

  private SwerveDrive m_swerve = SwerveDrive.getInstance();
  private PathPlannerTrajectory m_autoTrajectory;
  private PathPlannerPath m_autoPath = null;

  private RARHolonomicDriveController k_driveController = new RARHolonomicDriveController(
      new PIDConstants(RobotConstants.config.AutoAim.Translation.k_P,
          RobotConstants.config.AutoAim.Translation.k_I,
          RobotConstants.config.AutoAim.Translation.k_D),
      new PIDConstants(
          RobotConstants.config.AutoAim.Rotation.k_P,
          RobotConstants.config.AutoAim.Rotation.k_I,
          RobotConstants.config.AutoAim.Rotation.k_D),
      RobotConstants.config.Auto.k_maxModuleSpeed,
      Units.inchesToMeters(Math.sqrt(2) * (RobotConstants.config.Robot.k_width / 2)));

  private final Timer m_runningTimer = new Timer();

  public DriveTrajectoryTask(String pathName) {
    try {
      m_autoPath = PathPlannerPath.fromPathFile(pathName);

      if (DriverStation.getAlliance().get() == Alliance.Red) {
        RobotTelemetry.print("Translating path for Red Alliance!");
        m_autoPath = m_autoPath.flipPath();
      }
    } catch (Exception ex) {
      DriverStation.reportError("Unable to load PathPlanner trajectory: " + pathName, ex.getStackTrace());
    }
  }

  @Override
  public void prepare() {
    RobotTelemetry.print("Auto trajectory start");
    m_runningTimer.reset();
    m_runningTimer.start();

    // We probably want to reset this to the pose's starting rotation
    m_swerve.setAllianceGyroAngleAdjustment();

    m_autoTrajectory = m_autoPath.getTrajectory(
        new ChassisSpeeds(),
        m_swerve.getGyro().getRotation2d());

    if (k_debugPath) {
      Trajectory adjustedTrajectory = TrajectoryGenerator.generateTrajectory(
          m_autoPath.getPathPoses(),
          new TrajectoryConfig(
              m_autoPath.getGlobalConstraints().getMaxVelocityMps(),
              m_autoPath.getGlobalConstraints().getMaxAccelerationMpsSq()));

      Logger.recordOutput("Auto/DriveTrajectory/TargetTrajectory", adjustedTrajectory);
    }
    Logger.recordOutput("Auto/DriveTrajectory/StartingTargetPose", getStartingPose());

    // TODO: we probably want to do this all the time?
    if (!m_swerve.hasSetPose()) {
      if (RobotConstants.getInstance().getRobotType() == RobotType.AMADEUS) {
        // m_swerve.resetOdometry(getStartingPose());
      }
    }

    m_swerve.clearTurnPIDAccumulation();
    RobotTelemetry.print("Running path for " + DriverStation.getAlliance().get().toString());
  }

  @Override
  public void update() {
    log(true);

    if (m_autoTrajectory != null) {
      State goal = m_autoTrajectory.sample(m_runningTimer.get());
      Pose2d pose = m_swerve.getPose();

      goal.targetHolonomicRotation = getRotationProvider(goal.targetHolonomicRotation.getDegrees());

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

  private Rotation2d getRotationProvider(double targetRotation) {
    boolean isReadyToAutoTarget = Intake.getInstance().isAtStow() && Shooter.getInstance().isShooterReady();

    if (isReadyToAutoTarget) {
      return AllianceHelpers.getAllianceSpeakerRotationTarget();
    } else {
      return new Rotation2d(Math.toRadians(targetRotation));
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

    RobotTelemetry.print("Auto trajectory done");
    m_swerve.drive(0, 0, 0, true);
    // m_swerve.setRotationTarget(m_swerve.getPose().getRotation());
  }
}
