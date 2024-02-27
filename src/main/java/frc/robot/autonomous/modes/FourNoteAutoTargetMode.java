package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.ParallelTask;

public class FourNoteAutoTargetMode extends AutoModeBase {
  public void queueTasks() {
    // Note 1 (preload)
    // queueTask(new ShooterTask(ShooterPivotTarget.SUBWOOFER,
    // ShooterSpeedTarget.MAX));
    // queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    // queueTask(new WaitTask(Constants.Auto.Timing.k_shootRevTime));
    // queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    // queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));
    // queueTask(new ShooterTask(ShooterPivotTarget.PODIUM,
    // ShooterSpeedTarget.MAX));

    // Note 2 (MidRing)
    queueTask(new ParallelTask(
        // new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE),
        new DriveTrajectoryTask("Shoot, MidRing")));
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
    // new AutoTargetTask(getAllianceSpeakerPose())));
    // queueTask(new WaitTask(Constants.Auto.Timing.k_intakeBounceTime));
    // queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    // queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));

    // // Note 3 (PodiumRing)
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE),
    // new DriveTrajectoryTask("MidRing, PodiumRing")));
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
    // new AutoTargetTask(getAllianceSpeakerPose())));
    // queueTask(new WaitTask(Constants.Auto.Timing.k_intakeBounceTime));
    // queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    // queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));

    // // Note 4 (BotRing)
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE),
    // new DriveTrajectoryTask("PodiumRing, BotRing")));
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
    // new AutoTargetTask(getAllianceSpeakerPose())));
    // queueTask(new WaitTask(Constants.Auto.Timing.k_intakeBounceTime));
    // queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    // queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));

    // // Done
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE),
    // new ShooterTask(ShooterPivotTarget.SUBWOOFER, ShooterSpeedTarget.OFF),
    // new DriveTrajectoryTask("BotRing,Mid")));

    queueTask(new DriveForwardTask(0, 0));
  }
}
