package frc.robot.autonomous.modes;

import java.util.ArrayList;

import frc.robot.AllianceHelpers;
import frc.robot.autonomous.tasks.AutoTargetTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.Task;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public abstract class AutoModeBase {
  private ArrayList<Task> m_tasks;
  private SwerveDrive m_swerve = SwerveDrive.getInstance();

  public AutoModeBase() {
    m_tasks = new ArrayList<>();
    m_swerve.m_visionConstants = getVisionTargetConstants();
  }

  public Task getNextTask() {
    // Pop the first task off the list and return it
    try {
      return m_tasks.remove(0);
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public VisionConstants getVisionTargetConstants() {
    return RobotConstants.config.Vision.defaultAutoVisionConstants;
  }

  public void queueTask(Task task) {
    m_tasks.add(task);
  }

  public void queueShoot() {
    queueTask(new ParallelTask(
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER),
        new WaitTask(RobotConstants.config.Auto.Timing.k_shootFeedTime)));
  }

  public void queueDriveAndIntake(String path) {
    queueTask(new ParallelTask(
        new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE),
        new SequentialTask(
            new WaitTask(RobotConstants.config.Auto.Timing.k_intakeDeployTime),
            new DriveTrajectoryTask(path))));
  }

  public void queueDriveQuickAndIntake(String path) {
    queueTask(new ParallelTask(
        new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE),
        new DriveTrajectoryTask(path)));
  }

  public void queueDrive(String path) {
    queueTask(new DriveTrajectoryTask(path));
  }

  public void queueAutoTarget() {
    queueTask(new ParallelTask(
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
        new AutoTargetTask(AllianceHelpers.getAllianceSpeakerPose3d()),
        new WaitTask(RobotConstants.config.Auto.Timing.k_intakeBounceTime)));

    queueTask(new DriveForwardTask(0, 0));
  }

  public void queueAutoTarget(double extraTime) {
    queueTask(new ParallelTask(
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
        new AutoTargetTask(AllianceHelpers.getAllianceSpeakerPose3d()),
        new WaitTask(extraTime)));

    queueTask(new DriveForwardTask(0, 0));
  }

  public void queueShooterSpinUp() {
    queueTask(new ParallelTask(
        new ShooterTask(ShooterPivotTarget.SUBWOOFER, ShooterSpeedTarget.MAX),
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE)));
  }

  public void queueEnd() {
    queueTask(new DriveForwardTask(0, 0));
  }

  public abstract void queueTasks();
}
