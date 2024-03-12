package frc.robot.autonomous.modes;

import frc.robot.AllianceHelpers;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.AutoTargetTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.Intake.IntakePivotTarget;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterPivotTarget;
import frc.robot.subsystems.Shooter.ShooterSpeedTarget;

public class CenterlineOutsideNoteMode extends AutoModeBase {
  public void queueTasks() {
    // Note one (preload)
    queueTask(new ShooterTask(ShooterPivotTarget.PODIUM, ShooterSpeedTarget.MAX));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
    queueTask(new WaitTask(Constants.Auto.Timing.k_shootRevTime));
    queueTask(new AutoTargetTask(AllianceHelpers.getAllianceSpeakerPose3d()));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));
    queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));

    // Note two
    queueTask(new ParallelTask(
        new DriveTrajectoryTask("CenterFieldMiddle, Center 4"),
        new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE)));
    queueTask(new WaitTask(0.5));
    queueTask(new ParallelTask(
        new DriveTrajectoryTask("Center 4, Shoot Stage Right"),
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE)));
    queueTask(new AutoTargetTask(AllianceHelpers.getAllianceSpeakerPose3d()));
    queueTask(new DriveForwardTask(0, 0));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));

    queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));

    // Note two
    queueTask(new ParallelTask(
        new DriveTrajectoryTask("Shoot Stage Right, Center 5"),
        new IntakeTask(IntakePivotTarget.GROUND, IntakeState.INTAKE)));
    queueTask(new WaitTask(0.5));
    queueTask(new ParallelTask(
        new DriveTrajectoryTask("Center 5, Shoot Stage Right"),
        new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE)));
    queueTask(new AutoTargetTask(AllianceHelpers.getAllianceSpeakerPose3d()));
    queueTask(new DriveForwardTask(0, 0));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.FEED_SHOOTER));

    queueTask(new WaitTask(Constants.Auto.Timing.k_shootFeedTime));
    queueTask(new IntakeTask(IntakePivotTarget.STOW, IntakeState.NONE));
  }
}
