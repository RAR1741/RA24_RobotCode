package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;

public class TestMode2 extends AutoModeBase {
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
    queueTask(new ShooterTask(ShooterPivotTarget.SPEAKER, ShooterSpeedTarget.HALF));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    queueTask(new WaitTask(3));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    queueTask(new WaitTask(2));

    // Note 2 (center)
    queueTask(new IntakeTask(IntakePivotTarget.GROUND, IntakeState.NONE));
    queueTask(new DriveTrajectoryTask("Shoot, MidRing"));
    queueTask(new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE));
    queueTask(new WaitTask(0.5));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    queueTask(new DriveTrajectoryTask("MidRing, Shoot"));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    queueTask(new WaitTask(2));

    // Done
    queueTask(new ShooterTask(ShooterPivotTarget.SPEAKER, ShooterSpeedTarget.OFF));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
  }
}
