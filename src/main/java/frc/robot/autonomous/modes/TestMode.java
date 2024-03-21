package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;

public class TestMode extends AutoModeBase {
  @Override
  public void queueTasks() {
    // queueTask(new ParallelTask(new DriveTrajectoryTask("pleasework")));
    // queueTask(new DriveTrajectoryTask("pleasegodihope"));

    // queueTask(new SequentialTask(
    // new DriveTrajectoryTask("Shoot, MidRing"),
    // new DriveTrajectoryTask("MidRing, Shoot")));

    // queueTask(new PointForwardTask());
    // queueTask(new WaitTask(0.5));
    // queueTask(new DriveTrajectoryTask("Shoot, MidRing"));

    // Note 1 (preload)
    // queueTask(new ShooterTask(ShooterPivotTarget.SPEAKER,
    // ShooterSpeedTarget.MAX));
    // queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    // queueTask(new WaitTask(5));
    // queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    // queueTask(new WaitTask(2));

    // Note 2 (center)
    // queueTask(new IntakeTask(IntakePivotTarget.GROUND, IntakeState.NONE));

    queueTask(new DriveTrajectoryTask("pleasegodihope"));
    queueTask(new DriveForwardTask(0,0));

    // queueTask(new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE));
    // queueTask(new WaitTask(0.5));
    // queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));

    // queueTask(new DriveTrajectoryTask("MidRing, Shoot"));

    // queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    // queueTask(new WaitTask(2));

    // Done
    // queueTask(new ShooterTask(ShooterPivotTarget.SPEAKER,
    // ShooterSpeedTarget.OFF));
    // queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
  }
}
