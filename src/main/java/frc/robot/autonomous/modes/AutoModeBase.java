package frc.robot.autonomous.modes;

import java.util.ArrayList;

import frc.robot.AllianceHelpers;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.AutoTargetTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.Task;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;

public abstract class AutoModeBase {
  private ArrayList<Task> m_tasks;

  public AutoModeBase() {
    m_tasks = new ArrayList<>();
  }

  public Task getNextTask() {
    // Pop the first task off the list and return it
    try {
      return m_tasks.remove(0);
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public void queueTask(Task task) {
    m_tasks.add(task);
  }

  public void queueShoot() {
    queueTask(new ParallelTask(
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER),
        new WaitTask(Constants.Auto.Timing.k_shootFeedTime)));
  }

  public void queueDriveAndIntake(String path) {
    queueTask(new ParallelTask(
        new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE),
        new SequentialTask(
            new WaitTask(Constants.Auto.Timing.k_intakeDeployTime),
            new DriveTrajectoryTask(path))));
  }

  public void queueDrive(String path) {
    queueTask(new DriveTrajectoryTask(path));
  }

  public void queueAutoTarget() {
    queueTask(new ParallelTask(
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
        new AutoTargetTask(AllianceHelpers.getAllianceSpeakerPose3d()),
        new WaitTask(Constants.Auto.Timing.k_intakeBounceTime)));

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
